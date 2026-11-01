#include "rp/adapters/kdl_kinematics.h"
#include "rp/adapters/tesseract_planner.h"
#include "rp/api/scene_io.h"
#include "rp/core/collision/collision.h"
#include "rp/core/kinematics/kinematics.h"
#include "rp/planning/planner.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

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
  const rp::SceneConfig pose_config =
      rp::loadSceneConfig(std::string(RP_SOURCE_DIR) + "/scenes/pose_sequence.json");

  if (rp::isKdlAvailable()) {
    rp::KdlChainSolver kdl(config.scene.robot);
    check(kdl.valid(), "KDL chain solver builds");

    const rp::VecX home = rp::homeConfiguration(config.scene.robot);
    const rp::Isometry home_kdl = kdl.forward(home).tool;
    const rp::Isometry home_internal = rp::Kinematics::forward(config.scene.robot, home).tool;
    check((home_kdl.translation() - home_internal.translation()).norm() < 1e-6,
          "KDL FK position matches internal FK");
    check(orientationError(home_kdl.rotation(), home_internal.rotation()) < 1e-6,
          "KDL FK orientation matches internal FK");

    std::mt19937 generator(19);
    std::uniform_real_distribution<double> perturbation(-0.35, 0.35);
    for (int trial = 0; trial < 8; ++trial) {
      rp::VecX target_q = home;
      for (Eigen::Index i = 0; i < target_q.size(); ++i) {
        target_q[i] += perturbation(generator);
      }
      const rp::Isometry target = rp::Kinematics::forward(config.scene.robot, target_q).tool;
      rp::IKOptions options;
      const rp::IKResult ik = kdl.solve(target, home, options);
      check(ik.success, "KDL IK trial " + std::to_string(trial) + " succeeds");
      if (ik.success) {
        const rp::Isometry reached = rp::Kinematics::forward(config.scene.robot, ik.q).tool;
        check((reached.translation() - target.translation()).norm() < 1e-3,
              "KDL IK trial " + std::to_string(trial) + " position");
        check(orientationError(reached.rotation(), target.rotation()) < 1e-2,
              "KDL IK trial " + std::to_string(trial) + " orientation");
      }
    }
  } else {
    std::cout << "KDL unavailable, skipping KDL checks\n";
  }

  if (rp::isFclAvailable()) {
    rp::CollisionOptions internal_options;
    internal_options.backend = rp::CollisionBackend::Internal;
    internal_options.margin = config.options.collision_margin;
    internal_options.check_ground = config.scene.has_ground;
    internal_options.ground_z = config.scene.ground_z;

    rp::CollisionOptions fcl_options = internal_options;
    fcl_options.backend = rp::CollisionBackend::Fcl;

    const rp::CollisionChecker internal_checker(config.scene, internal_options);
    const rp::CollisionChecker fcl_checker(config.scene, fcl_options);
    const rp::VecX home = rp::homeConfiguration(config.scene.robot);
    check(internal_checker.checkConfiguration(home).collision ==
              fcl_checker.checkConfiguration(home).collision,
          "FCL and internal agree at home");
    check(!fcl_checker.checkConfiguration(home).collision, "FCL home configuration is free");

    std::mt19937 generator(31);
    std::uniform_real_distribution<double> perturbation(-0.30, 0.30);
    for (int trial = 0; trial < 10; ++trial) {
      rp::VecX q = home;
      for (Eigen::Index i = 0; i < q.size(); ++i) {
        q[i] += perturbation(generator);
      }
      const bool internal_collision = internal_checker.checkConfiguration(q).collision;
      const bool fcl_collision = fcl_checker.checkConfiguration(q).collision;
      check(internal_collision == fcl_collision,
            "FCL and internal collision result agree on trial " + std::to_string(trial));
    }
  } else {
    std::cout << "FCL unavailable, skipping FCL checks\n";
  }

  if (rp::isKdlAvailable() && rp::isFclAvailable()) {
    rp::PlanningOptions options = config.options;
    options.ik_backend = rp::IKBackend::Kdl;
    options.collision_backend = rp::CollisionBackend::Fcl;
    options.motion_backend = rp::MotionPlannerBackend::InternalRRTConnect;
    options.require_full_coverage = false;

    const rp::PlanResult surface_plan =
        rp::Planner::plan(config.scene, config.surface, config.scanner, options);
    check(surface_plan.success,
          "KDL + FCL surface plan succeeds: " + surface_plan.message);
    check(surface_plan.metrics.coverage_ratio >= 0.95,
          "KDL + FCL surface coverage is acceptable");

    rp::PlanningOptions pose_options = pose_config.options;
    pose_options.ik_backend = rp::IKBackend::Kdl;
    pose_options.collision_backend = rp::CollisionBackend::Fcl;
    pose_options.motion_backend = rp::MotionPlannerBackend::InternalRRTConnect;
    const rp::PlanResult pose_plan =
        rp::Planner::planPoseSequence(pose_config.scene, pose_config.poses, pose_options);
    check(pose_plan.success, "KDL + FCL pose sequence succeeds: " + pose_plan.message);
    check(pose_plan.pose_visits.size() == pose_config.poses.size(),
          "KDL + FCL visits all poses");
  }

  if (rp::isTesseractAvailable()) {
    rp::TesseractPlanner tesseract(config.scene);
    check(tesseract.valid(), "Tesseract environment builds: " + tesseract.error());
    std::vector<rp::PoseTarget> poses(pose_config.poses.begin(), pose_config.poses.begin() + 2);
    rp::PlanningOptions options = pose_config.options;
    options.motion_backend = rp::MotionPlannerBackend::TesseractOMPL;
    options.ompl_time_limit = 0.5;
    const rp::PlanResult plan =
        rp::Planner::planPoseSequence(pose_config.scene, poses, options);
    check(plan.success, "Tesseract pose sequence succeeds: " + plan.message);
    check(plan.pose_visits.size() == poses.size(), "Tesseract visits all poses");
  } else {
    std::cout << "Tesseract unavailable, skipping Tesseract checks\n";
  }

  if (failures != 0) {
    std::cerr << failures << " adapter checks failed\n";
    return 1;
  }
  std::cout << "test_adapters passed\n";
  return 0;
}
