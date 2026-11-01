#include "rp/core/kinematics/kinematics.h"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace rp {

namespace {

Isometry jointMotion(const Joint& joint, double value) {
  Isometry T = Isometry::Identity();
  if (joint.type == JointType::Revolute) {
    T.linear() = Eigen::AngleAxisd(value, joint.axis.normalized()).toRotationMatrix();
  } else {
    T.translation() = joint.axis.normalized() * value;
  }
  return T;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> geometricJacobian(const SerialChain& chain,
                                                           const FKResult& fk,
                                                           const VecX& q) {
  const auto n = static_cast<Eigen::Index>(chain.joints.size());
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, n);
  J.setZero();
  const Vec3 tool_position = fk.tool.translation();
  for (Eigen::Index i = 0; i < n; ++i) {
    const Joint& joint = chain.joints[static_cast<std::size_t>(i)];
    const Vec3 axis = fk.joint_axes[static_cast<std::size_t>(i)];
    if (joint.type == JointType::Revolute) {
      const Vec3 origin = fk.joint_origins[static_cast<std::size_t>(i)];
      J.block<3, 1>(0, i) = axis.cross(tool_position - origin);
      J.block<3, 1>(3, i) = axis;
    } else {
      J.block<3, 1>(0, i) = axis;
      J.block<3, 1>(3, i).setZero();
    }
  }
  (void)q;
  return J;
}

Vec3 orientationError(const Mat3& target, const Mat3& current) {
  const Mat3 delta = target * current.transpose();
  Eigen::AngleAxisd angle_axis(delta);
  return angle_axis.axis() * angle_axis.angle();
}

double costOf(const Vec3& position_error, const Vec3& rotation_error) {
  return position_error.norm() + rotation_error.norm();
}

}  // namespace

FKResult Kinematics::forward(const SerialChain& chain, const VecX& q) {
  const auto n = static_cast<Eigen::Index>(chain.joints.size());
  FKResult result;
  result.link_frames.resize(static_cast<std::size_t>(n));
  result.joint_origins.resize(static_cast<std::size_t>(n));
  result.joint_axes.resize(static_cast<std::size_t>(n));

  Isometry T = chain.base;
  for (Eigen::Index i = 0; i < n; ++i) {
    const Joint& joint = chain.joints[static_cast<std::size_t>(i)];
    T = T * joint.origin;
    result.joint_origins[static_cast<std::size_t>(i)] = T.translation();
    result.joint_axes[static_cast<std::size_t>(i)] = T.rotation() * joint.axis.normalized();
    T = T * jointMotion(joint, q[i]);
    result.link_frames[static_cast<std::size_t>(i)] = T;
  }
  result.tool = T * chain.tool;
  return result;
}

bool Kinematics::isWithinLimits(const SerialChain& chain, const VecX& q, double tolerance) {
  if (q.size() != static_cast<Eigen::Index>(chain.joints.size())) {
    return false;
  }
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    const Joint& joint = chain.joints[static_cast<std::size_t>(i)];
    if (q[i] < joint.lower - tolerance || q[i] > joint.upper + tolerance) {
      return false;
    }
    if (!std::isfinite(q[i])) {
      return false;
    }
  }
  return true;
}

VecX Kinematics::clampToLimits(const SerialChain& chain, const VecX& q) {
  VecX clamped = q;
  for (Eigen::Index i = 0; i < clamped.size(); ++i) {
    const Joint& joint = chain.joints[static_cast<std::size_t>(i)];
    clamped[i] = std::min(std::max(clamped[i], joint.lower), joint.upper);
  }
  return clamped;
}

IKResult Kinematics::solve(const SerialChain& chain, const Isometry& target, const VecX& seed,
                           const IKOptions& options) {
  IKResult result;
  const auto n = static_cast<Eigen::Index>(chain.joints.size());
  if (seed.size() != n) {
    return result;
  }

  VecX q = clampToLimits(chain, seed);
  double damping = options.initial_damping;

  FKResult fk = forward(chain, q);
  Vec3 position_error = target.translation() - fk.tool.translation();
  Vec3 rotation_error = orientationError(target.rotation(), fk.tool.rotation());
  double best_cost = costOf(position_error, rotation_error);
  VecX best_q = q;

  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    result.iterations = iteration + 1;
    if (position_error.norm() <= options.position_tolerance &&
        rotation_error.norm() <= options.orientation_tolerance) {
      break;
    }

    const auto J = geometricJacobian(chain, fk, q);
    Vec6 error;
    error.head<3>() = position_error;
    error.tail<3>() = rotation_error;

    bool improved = false;
    for (int attempt = 0; attempt < options.max_line_search; ++attempt) {
      const double lambda2 = damping * damping;
      Mat6 normal = J * J.transpose();
      normal.diagonal().array() += lambda2;

      const Vec6 y = normal.ldlt().solve(error);
      if (!y.allFinite()) {
        damping = std::min(damping * 4.0, options.max_damping);
        continue;
      }
      VecX dq = J.transpose() * y;
      const double step_norm = dq.norm();
      if (step_norm > options.max_step_norm) {
        dq *= options.max_step_norm / step_norm;
      }
      if (dq.norm() < 1e-12) {
        break;
      }

      VecX candidate = clampToLimits(chain, q + dq);
      const FKResult candidate_fk = forward(chain, candidate);
      const Vec3 candidate_position =
          target.translation() - candidate_fk.tool.translation();
      const Vec3 candidate_rotation =
          orientationError(target.rotation(), candidate_fk.tool.rotation());
      const double candidate_cost = costOf(candidate_position, candidate_rotation);

      if (candidate_cost < best_cost) {
        best_cost = candidate_cost;
        best_q = candidate;
      }
      if (candidate_cost < costOf(position_error, rotation_error)) {
        q = candidate;
        fk = candidate_fk;
        position_error = candidate_position;
        rotation_error = candidate_rotation;
        damping = std::max(damping * 0.5, options.min_damping);
        improved = true;
        break;
      }
      damping = std::min(damping * 4.0, options.max_damping);
    }

    if (!improved) {
      break;
    }
  }

  const FKResult final_fk = forward(chain, best_q);
  result.q = best_q;
  result.position_error = (target.translation() - final_fk.tool.translation()).norm();
  result.orientation_error = orientationError(target.rotation(), final_fk.tool.rotation()).norm();
  result.success = result.position_error <= options.position_tolerance &&
                   result.orientation_error <= options.orientation_tolerance &&
                   isWithinLimits(chain, best_q);
  return result;
}

IKResult Kinematics::solveMultiSeed(const SerialChain& chain, const Isometry& target,
                                    const std::vector<VecX>& seeds,
                                    const IKOptions& options) {
  IKResult best;
  best.position_error = 1e9;
  best.orientation_error = 1e9;
  for (const VecX& seed : seeds) {
    if (seed.size() != static_cast<Eigen::Index>(chain.joints.size())) {
      continue;
    }
    const IKResult candidate = solve(chain, target, seed, options);
    if (candidate.success) {
      return candidate;
    }
    const double score = candidate.position_error + candidate.orientation_error;
    const double best_score = best.position_error + best.orientation_error;
    if (score < best_score) {
      best = candidate;
    }
  }
  return best;
}

std::vector<VecX> Kinematics::defaultSeeds(const SerialChain& chain) {
  const auto n = static_cast<Eigen::Index>(chain.joints.size());
  std::vector<VecX> seeds;
  VecX home = homeConfiguration(chain);
  seeds.push_back(home);

  VecX zeros = VecX::Zero(n);
  seeds.push_back(zeros);

  VecX alternating(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    alternating[i] = (i % 2 == 0) ? 0.5 : -0.5;
  }
  seeds.push_back(alternating);

  VecX bent(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    bent[i] = (i % 3 == 0) ? -0.7 : ((i % 3 == 1) ? 0.9 : -0.4);
  }
  seeds.push_back(bent);

  VecX bent2(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    bent2[i] = (i % 3 == 0) ? 0.7 : ((i % 3 == 1) ? -0.9 : 0.4);
  }
  seeds.push_back(bent2);

  VecX folded(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    folded[i] = (i % 2 == 0) ? 0.9 : -1.2;
  }
  seeds.push_back(folded);

  VecX folded2 = -folded;
  seeds.push_back(folded2);

  VecX base_left(n);
  base_left.setZero();
  if (n > 0) {
    base_left[0] = 0.6;
  }
  if (n > 1) {
    base_left[1] = -0.5;
  }
  if (n > 2) {
    base_left[2] = 0.9;
  }
  seeds.push_back(base_left);

  VecX base_right(n);
  base_right.setZero();
  if (n > 0) {
    base_right[0] = -0.6;
  }
  if (n > 1) {
    base_right[1] = -0.5;
  }
  if (n > 2) {
    base_right[2] = 0.9;
  }
  seeds.push_back(base_right);

  return seeds;
}

}  // namespace rp
