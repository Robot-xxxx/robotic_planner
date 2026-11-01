#include "rp/adapters/kdl_kinematics.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#ifdef RP_WITH_KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#endif

namespace rp {

namespace {

#ifdef RP_WITH_KDL

KDL::Frame toKdlFrame(const Isometry& transform) {
  const Mat3& rotation = transform.linear();
  KDL::Rotation kdl_rotation(rotation(0, 0), rotation(0, 1), rotation(0, 2), rotation(1, 0),
                             rotation(1, 1), rotation(1, 2), rotation(2, 0), rotation(2, 1),
                             rotation(2, 2));
  const Vec3 translation = transform.translation();
  return KDL::Frame(kdl_rotation, KDL::Vector(translation.x(), translation.y(), translation.z()));
}

Isometry fromKdlFrame(const KDL::Frame& frame) {
  Isometry result = Isometry::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.linear()(row, column) = frame.M(row, column);
    }
  }
  result.translation() = Vec3(frame.p.x(), frame.p.y(), frame.p.z());
  return result;
}

KDL::Vector toKdlVector(const Vec3& vector) {
  return KDL::Vector(vector.x(), vector.y(), vector.z());
}

#endif

}  // namespace

struct KdlChainSolver::Impl {
  std::string error;
  bool valid = false;
  SerialChain serial_chain;
#ifdef RP_WITH_KDL
  KDL::Chain chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_velocity_solver;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_limited_solver;
  KDL::JntArray q_min;
  KDL::JntArray q_max;
#endif
};

bool isKdlAvailable() {
#ifdef RP_WITH_KDL
  return true;
#else
  return false;
#endif
}

KdlChainSolver::KdlChainSolver(const SerialChain& chain) : impl_(std::make_unique<Impl>()) {
#ifdef RP_WITH_KDL
  std::string error;
  if (!validateChain(chain, &error)) {
    impl_->error = "cannot build KDL chain: " + error;
    return;
  }

  impl_->chain.addSegment(
      KDL::Segment("base", KDL::Joint(KDL::Joint::None), toKdlFrame(chain.base)));

  for (const Joint& joint : chain.joints) {
    const KDL::Vector axis = toKdlVector(joint.axis.normalized());
    const KDL::Joint::JointType type =
        (joint.type == JointType::Revolute) ? KDL::Joint::RotAxis : KDL::Joint::TransAxis;
    impl_->chain.addSegment(KDL::Segment("origin_" + joint.name, KDL::Joint(KDL::Joint::None),
                                        toKdlFrame(joint.origin)));
    impl_->chain.addSegment(KDL::Segment(
        joint.name, KDL::Joint(joint.name, KDL::Vector(0.0, 0.0, 0.0), axis, type)));
  }

  impl_->chain.addSegment(
      KDL::Segment("tool", KDL::Joint(KDL::Joint::None), toKdlFrame(chain.tool)));

  impl_->serial_chain = chain;
  impl_->fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(impl_->chain);
  impl_->ik_solver = std::make_unique<KDL::ChainIkSolverPos_LMA>(impl_->chain);
  const unsigned int joint_count = impl_->chain.getNrOfJoints();
  impl_->q_min.resize(joint_count);
  impl_->q_max.resize(joint_count);
  for (unsigned int i = 0; i < joint_count; ++i) {
    impl_->q_min(i) = chain.joints[i].lower;
    impl_->q_max(i) = chain.joints[i].upper;
  }
  impl_->ik_velocity_solver = std::make_unique<KDL::ChainIkSolverVel_pinv>(impl_->chain);
  impl_->ik_limited_solver = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      impl_->chain, impl_->q_min, impl_->q_max, *impl_->fk_solver, *impl_->ik_velocity_solver);
  impl_->valid = true;
#else
  (void)chain;
  impl_->error = "KDL support was not enabled at build time";
#endif
}

KdlChainSolver::~KdlChainSolver() = default;
KdlChainSolver::KdlChainSolver(KdlChainSolver&&) noexcept = default;
KdlChainSolver& KdlChainSolver::operator=(KdlChainSolver&&) noexcept = default;

bool KdlChainSolver::valid() const { return impl_ != nullptr && impl_->valid; }

const std::string& KdlChainSolver::error() const {
  static const std::string empty;
  if (!impl_) {
    return empty;
  }
  return impl_->error;
}

FKResult KdlChainSolver::forward(const VecX& q) const {
  if (!valid()) {
    return {};
  }
#ifdef RP_WITH_KDL
  const unsigned int joint_count = impl_->chain.getNrOfJoints();
  if (q.size() != static_cast<Eigen::Index>(joint_count)) {
    return {};
  }
  KDL::JntArray joint_positions(joint_count);
  for (unsigned int i = 0; i < joint_count; ++i) {
    joint_positions(i) = q[static_cast<Eigen::Index>(i)];
  }
  KDL::Frame frame;
  if (impl_->fk_solver->JntToCart(joint_positions, frame) < 0) {
    return {};
  }

  FKResult result;
  result.tool = fromKdlFrame(frame);
  return result;
#else
  (void)q;
  return {};
#endif
}

IKResult KdlChainSolver::solve(const Isometry& target, const VecX& seed,
                               const IKOptions& options) const {
  IKResult result;
  if (!valid()) {
    return result;
  }
#ifdef RP_WITH_KDL
  const unsigned int joint_count = impl_->chain.getNrOfJoints();
  if (seed.size() != static_cast<Eigen::Index>(joint_count)) {
    return result;
  }

  KDL::JntArray q_init(joint_count);
  for (unsigned int i = 0; i < joint_count; ++i) {
    q_init(i) = seed[static_cast<Eigen::Index>(i)];
  }
  const KDL::Frame kdl_target = toKdlFrame(target);

  auto evaluate = [&](const KDL::JntArray& q_raw) {
    IKResult candidate;
    candidate.q.resize(static_cast<Eigen::Index>(joint_count));
    for (unsigned int i = 0; i < joint_count; ++i) {
      candidate.q[static_cast<Eigen::Index>(i)] = q_raw(i);
    }
    if (!isFinite(candidate.q)) {
      return candidate;
    }
    const bool within_limits = Kinematics::isWithinLimits(impl_->serial_chain, candidate.q, 1e-9);
    candidate.q = Kinematics::clampToLimits(impl_->serial_chain, candidate.q);
    const FKResult fk = Kinematics::forward(impl_->serial_chain, candidate.q);
    candidate.position_error = (target.translation() - fk.tool.translation()).norm();
    const Mat3 rotation_error = target.rotation() * fk.tool.rotation().transpose();
    Eigen::AngleAxisd angle_axis(rotation_error);
    candidate.orientation_error = std::abs(angle_axis.angle());
    candidate.success = within_limits &&
                        candidate.position_error <= options.position_tolerance &&
                        candidate.orientation_error <= options.orientation_tolerance;
    return candidate;
  };

  KDL::JntArray q_out(joint_count);
  impl_->ik_solver->CartToJnt(q_init, kdl_target, q_out);
  result = evaluate(q_out);
  if (result.success) {
    return result;
  }

  KDL::JntArray q_limited(joint_count);
  impl_->ik_limited_solver->CartToJnt(q_init, kdl_target, q_limited);
  IKResult limited = evaluate(q_limited);
  if (limited.success) {
    return limited;
  }

  const double result_score = result.position_error + result.orientation_error;
  const double limited_score = limited.position_error + limited.orientation_error;
  return (limited_score < result_score) ? limited : result;
#else
  (void)target;
  (void)seed;
  (void)options;
  return result;
#endif
}

}  // namespace rp
