#include "rp/api/scene_io.h"
#include "rp/core/collision/collision.h"
#include "rp/core/kinematics/kinematics.h"
#include "rp/planning/planner.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <set>
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

void validatePlan(const rp::SceneConfig& config, const rp::PlanResult& plan,
                  const std::string& label) {
  check(plan.success, label + ": plan succeeds: " + plan.message);
  check(plan.task_type == "pose_sequence", label + ": task type");
  check(plan.metrics.poses_total == static_cast<int>(config.poses.size()),
        label + ": all input poses counted");
  check(plan.metrics.poses_reachable == static_cast<int>(config.poses.size()),
        label + ": all poses reachable");
  check(plan.pose_visits.size() == config.poses.size(), label + ": all poses visited");
  check(plan.metrics.collision_free, label + ": collision free");
  check(plan.metrics.order_valid, label + ": order valid");
  check(plan.metrics.total_time_s > 0.0, label + ": positive total time");
  check(plan.metrics.dwell_time_s > 0.0, label + ": dwell time recorded");
  check(plan.metrics.tcp_path_m > 0.0, label + ": TCP path recorded");

  std::set<int> visited_indices;
  for (const rp::PlannedPoseVisit& visit : plan.pose_visits) {
    visited_indices.insert(visit.input_index);
    check(visit.q.size() == static_cast<Eigen::Index>(config.scene.robot.joints.size()),
          label + ": visit dimension");
    check(rp::Kinematics::isWithinLimits(config.scene.robot, visit.q),
          label + ": visit respects joint limits");

    const rp::Isometry reached = rp::Kinematics::forward(config.scene.robot, visit.q).tool;
    const rp::PoseTarget& target = config.poses[static_cast<std::size_t>(visit.input_index)];
    check((reached.translation() - target.pose.translation()).norm() < 1e-3,
          label + ": visit reaches target position");
    check(orientationError(reached.rotation(), target.pose.rotation()) < 1e-2,
          label + ": visit reaches target orientation");
    check(visit.dwell_time >= 0.0, label + ": dwell time non-negative");
  }
  check(visited_indices.size() == config.poses.size(), label + ": each pose visited once");

  if (config.options.return_home) {
    check(plan.transitions.size() == plan.pose_visits.size() + 1,
          label + ": transitions include return home");
  } else {
    check(plan.transitions.size() == plan.pose_visits.size(),
          label + ": one transfer per visited pose");
  }

  const rp::VecX home = rp::homeConfiguration(config.scene.robot);
  if (!plan.transitions.empty() && !plan.pose_visits.empty()) {
    check((plan.transitions.front().waypoints.front().q - home).norm() < 1e-4,
          label + ": first transfer starts at home");
    for (std::size_t i = 0; i < plan.pose_visits.size(); ++i) {
      const rp::PlannedTransition& transition = plan.transitions[i];
      check(transition.to_pose == plan.pose_visits[i].input_index,
            label + ": transfer reaches the requested pose");
      if (i > 0) {
        check(transition.from_pose == plan.pose_visits[i - 1].input_index,
              label + ": transfer starts at the previous pose");
        check((transition.waypoints.front().q - plan.pose_visits[i - 1].q).norm() < 1e-4,
              label + ": transfer configuration continuity");
      }
      check((transition.waypoints.back().q - plan.pose_visits[i].q).norm() < 1e-4,
            label + ": transfer ends at the visit configuration");
    }
    if (config.options.return_home) {
      check(plan.transitions.back().to_pose == -1, label + ": final transfer returns home");
      check((plan.transitions.back().waypoints.back().q - home).norm() < 1e-4,
            label + ": final transfer ends at home");
    }
  }

  rp::CollisionOptions collision_options;
  collision_options.margin = config.options.collision_margin;
  collision_options.check_ground = config.scene.has_ground;
  collision_options.ground_z = config.scene.ground_z;
  const rp::CollisionChecker checker(config.scene, collision_options);
  for (const rp::PlannedTransition& transition : plan.transitions) {
    for (std::size_t i = 1; i < transition.waypoints.size(); ++i) {
      check(!checker.checkTransition(transition.waypoints[i - 1].q, transition.waypoints[i].q,
                                     config.options.collision_step)
                 .collision,
            label + ": transfer segment collision free");
    }
  }

  const std::string json = rp::planResultToJson(config, plan);
  const auto parsed = nlohmann::json::parse(json);
  check(parsed.value("task", "") == "pose_sequence", label + ": JSON task type");
  check(parsed.contains("pose_visits") && parsed["pose_visits"].size() == config.poses.size(),
        label + ": JSON pose visits");
  check(parsed["metrics"].value("poses_reachable", 0) ==
            static_cast<int>(config.poses.size()),
        label + ": JSON reachable pose count");
}

}  // namespace

int main() {
  const std::string path = std::string(RP_SOURCE_DIR) + "/scenes/pose_sequence.json";
  rp::SceneConfig config = rp::loadSceneConfig(path);
  check(config.task == rp::TaskType::PoseSequence, "config parses pose sequence task");
  check(config.poses.size() == 10, "config contains ten poses");

  rp::PlanningOptions internal = config.options;
  internal.motion_backend = rp::MotionPlannerBackend::InternalRRTConnect;
  const rp::PlanResult internal_plan =
      rp::Planner::planPoseSequence(config.scene, config.poses, internal);
  validatePlan(config, internal_plan, "internal");

  rp::PlanningOptions order_preserved = internal;
  order_preserved.optimize_order = false;
  const rp::PlanResult ordered_plan =
      rp::Planner::planPoseSequence(config.scene, config.poses, order_preserved);
  validatePlan(config, ordered_plan, "input_order");
  for (std::size_t i = 0; i < ordered_plan.pose_visits.size(); ++i) {
    check(ordered_plan.pose_visits[i].input_index == static_cast<int>(i),
          "input order is preserved when optimization is disabled");
  }

  rp::PlanningOptions with_return = internal;
  with_return.return_home = true;
  const rp::PlanResult return_plan =
      rp::Planner::planPoseSequence(config.scene, config.poses, with_return);
  rp::SceneConfig return_config = config;
  return_config.options.return_home = true;
  validatePlan(return_config, return_plan, "return_home");

  rp::CollisionOptions random_collision_options;
  random_collision_options.margin = config.options.collision_margin;
  random_collision_options.check_ground = config.scene.has_ground;
  random_collision_options.ground_z = config.scene.ground_z;
  const rp::CollisionChecker random_checker(config.scene, random_collision_options);
  std::mt19937 generator(42);
  std::uniform_real_distribution<double> perturbation(-0.40, 0.40);
  std::vector<rp::PoseTarget> random_poses;
  for (int attempt = 0; attempt < 500 && random_poses.size() < 10; ++attempt) {
    rp::VecX q = rp::homeConfiguration(config.scene.robot);
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      q[i] += perturbation(generator);
    }
    if (random_checker.collides(q)) {
      continue;
    }
    rp::PoseTarget target;
    target.name = "random_" + std::to_string(random_poses.size());
    target.pose = rp::Kinematics::forward(config.scene.robot, q).tool;
    random_poses.push_back(target);
  }
  check(random_poses.size() == 10, "generated ten random collision-free poses");
  rp::SceneConfig random_config = config;
  random_config.poses = random_poses;
  rp::PlanningOptions random_options = internal;
  random_options.require_all_poses = true;
  const rp::PlanResult random_plan =
      rp::Planner::planPoseSequence(config.scene, random_poses, random_options);
  validatePlan(random_config, random_plan, "random_6d_poses");

  std::vector<rp::PoseTarget> with_unreachable = config.poses;
  rp::PoseTarget far_pose;
  far_pose.name = "far";
  far_pose.pose = rp::makeIsometry(rp::Vec3(2.0, 0.0, 1.0),
                                  rp::rotationFromRpyDegrees(0.0, 90.0, 0.0));
  with_unreachable.push_back(far_pose);

  rp::PlanningOptions strict = internal;
  strict.require_all_poses = true;
  const rp::PlanResult strict_plan =
      rp::Planner::planPoseSequence(config.scene, with_unreachable, strict);
  check(!strict_plan.success, "unreachable pose fails a strict sequence");
  check(strict_plan.metrics.poses_total == 11, "strict sequence counts all poses");
  check(strict_plan.metrics.poses_reachable == 10, "strict sequence plans reachable subset");
  check(!strict_plan.warnings.empty(), "strict sequence reports unreachable pose");

  rp::PlanningOptions partial = internal;
  partial.require_all_poses = false;
  const rp::PlanResult partial_plan =
      rp::Planner::planPoseSequence(config.scene, with_unreachable, partial);
  check(partial_plan.success, "partial sequence succeeds with reachable subset");
  check(partial_plan.pose_visits.size() == 10, "partial sequence visits reachable poses");

  if (rp::isOmplAvailable()) {
    rp::PlanningOptions ompl = config.options;
    ompl.motion_backend = rp::MotionPlannerBackend::OmplRRTConnect;
    ompl.ompl_time_limit = 0.5;
    const rp::PlanResult ompl_plan =
        rp::Planner::planPoseSequence(config.scene, config.poses, ompl);
    rp::SceneConfig ompl_config = config;
    ompl_config.options = ompl;
    validatePlan(ompl_config, ompl_plan, "ompl_rrtconnect");
  }

  if (failures != 0) {
    std::cerr << failures << " pose sequence checks failed\n";
    return 1;
  }
  std::cout << "test_pose_sequence passed\n";
  return 0;
}
