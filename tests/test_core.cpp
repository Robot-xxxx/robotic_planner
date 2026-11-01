#include "rp/api/scene_io.h"
#include "rp/core/collision/collision.h"
#include "rp/core/kinematics/kinematics.h"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <random>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    ++failures;
  }
}

double orientationError(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  const Eigen::Matrix3d delta = a * b.transpose();
  Eigen::AngleAxisd angle_axis(delta);
  return std::abs(angle_axis.angle());
}

}  // namespace

int main() {
  const rp::SceneConfig config = rp::loadSceneConfig(RP_DEFAULT_CONFIG);
  const rp::SerialChain& chain = config.scene.robot;

  std::string error;
  check(rp::validateChain(chain, &error), "chain validation: " + error);
  check(chain.joints.size() == 6, "demo chain has six joints");

  const rp::VecX home = rp::homeConfiguration(chain);
  const rp::FKResult home_fk = rp::Kinematics::forward(chain, home);
  check(home_fk.tool.translation().allFinite(), "home FK is finite");

  rp::VecX outside = home;
  outside[0] = 10.0;
  check(!rp::Kinematics::isWithinLimits(chain, outside), "joint limit detection");
  const rp::VecX clamped = rp::Kinematics::clampToLimits(chain, outside);
  check(std::abs(clamped[0] - chain.joints[0].upper) < 1e-12, "joint limit clamping");

  std::mt19937 generator(7);
  std::uniform_real_distribution<double> distribution(-0.45, 0.45);
  const std::vector<rp::VecX> seeds = rp::Kinematics::defaultSeeds(chain);

  for (int trial = 0; trial < 8; ++trial) {
    rp::VecX target_q = home;
    for (Eigen::Index i = 0; i < target_q.size(); ++i) {
      target_q[i] += distribution(generator);
    }
    const rp::Isometry target = rp::Kinematics::forward(chain, target_q).tool;
    const rp::IKResult result = rp::Kinematics::solveMultiSeed(chain, target, seeds);
    check(result.success, "IK round-trip " + std::to_string(trial) + " succeeds");
    if (result.success) {
      const rp::Isometry reached = rp::Kinematics::forward(chain, result.q).tool;
      check((reached.translation() - target.translation()).norm() < 1e-3,
            "IK round-trip " + std::to_string(trial) + " position");
      check(orientationError(reached.rotation(), target.rotation()) < 1e-2,
            "IK round-trip " + std::to_string(trial) + " orientation");
    }
  }

  rp::Geometry capsule;
  capsule.type = rp::Geometry::Type::Capsule;
  capsule.a = rp::Vec3(0.0, 0.0, 0.0);
  capsule.b = rp::Vec3(0.0, 0.0, 1.0);
  capsule.radius = 0.1;

  rp::Geometry box;
  box.type = rp::Geometry::Type::Box;
  box.center = rp::Vec3(1.0, 0.0, 0.5);
  box.half_extents = rp::Vec3(0.5, 0.5, 0.5);

  const double distance = rp::geometryDistance(capsule, rp::Isometry::Identity(), box,
                                               rp::Isometry::Identity());
  check(std::abs(distance - 0.4) < 1e-3, "capsule-box distance");

  const double ground_distance =
      rp::geometryGroundDistance(capsule, rp::Isometry::Identity(), 0.0);
  check(std::abs(ground_distance + 0.1) < 1e-6, "capsule-ground distance");

  const rp::CollisionResult self = rp::CollisionChecker(config.scene).checkConfiguration(home);
  check(!self.collision, "home configuration is collision free");

  if (failures != 0) {
    std::cerr << failures << " core checks failed\n";
    return 1;
  }
  std::cout << "test_core passed\n";
  return 0;
}
