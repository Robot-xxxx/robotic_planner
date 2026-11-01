#include "rp/api/scene_io.h"
#include "rp/core/collision/collision.h"
#include "rp/core/kinematics/kinematics.h"
#include "rp/planning/planner.h"

#include <cmath>
#include <iostream>
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

void validatePlan(const rp::SceneConfig& config, const rp::PlanResult& plan,
                  const std::string& label) {
  check(plan.success, label + ": plan succeeds: " + plan.message);
  check(plan.metrics.coverage_ratio >= 0.999, label + ": full coverage");
  check(plan.metrics.samples_feasible == plan.metrics.samples_total,
        label + ": all samples are feasible");
  check(plan.metrics.collision_free, label + ": plan is collision free");
  check(plan.metrics.order_valid, label + ": order is valid");
  check(!plan.runs.empty(), label + ": runs are present");
  check(plan.transitions.size() == plan.runs.size(),
        label + ": one transfer per run");

  const rp::VecX home = rp::homeConfiguration(config.scene.robot);
  if (!plan.transitions.empty() && !plan.runs.empty()) {
    const rp::PlannedTransition& first = plan.transitions.front();
    if (!first.waypoints.empty()) {
      check((first.waypoints.front().q - home).norm() < 1e-4,
            label + ": first transfer starts at home");
      check((first.waypoints.back().q - plan.runs.front().waypoints.front().q).norm() < 1e-4,
            label + ": first transfer reaches the first run");
    }
  }

  for (std::size_t i = 1; i < plan.transitions.size(); ++i) {
    const rp::PlannedTransition& transition = plan.transitions[i];
    if (transition.waypoints.empty()) {
      continue;
    }
    check((transition.waypoints.front().q - plan.runs[i - 1].waypoints.back().q).norm() < 1e-4,
          label + ": transfer " + std::to_string(i) + " starts at the previous run");
    check((transition.waypoints.back().q - plan.runs[i].waypoints.front().q).norm() < 1e-4,
          label + ": transfer " + std::to_string(i) + " reaches the next run");
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
            label + ": transfer segment is collision free");
    }
  }
}

}  // namespace

int main() {
  if (!rp::isOmplAvailable()) {
    std::cout << "test_ompl skipped: OMPL is not available\n";
    return 0;
  }

  const rp::SceneConfig config =
      rp::loadSceneConfig(std::string(RP_SOURCE_DIR) + "/scenes/small_patch.json");

  const std::vector<rp::MotionPlannerBackend> backends = {
      rp::MotionPlannerBackend::OmplRRTConnect,
      rp::MotionPlannerBackend::OmplRRTStar,
      rp::MotionPlannerBackend::OmplBITstar,
  };

  for (const rp::MotionPlannerBackend backend : backends) {
    rp::PlanningOptions options = config.options;
    options.motion_backend = backend;
    options.ompl_time_limit = 0.5;
    options.ompl_seed = 7;
    options.ompl_simplify = true;
    options.require_full_coverage = true;

    const rp::PlanResult plan =
        rp::Planner::plan(config.scene, config.surface, config.scanner, options);
    const std::string label = rp::motionBackendName(backend);
    validatePlan(config, plan, label);
    std::cout << label << ": coverage=" << plan.metrics.coverage_ratio
              << " time=" << plan.metrics.total_time_s << "s path=" << plan.metrics.tcp_path_m
              << "m runs=" << plan.metrics.runs_planned << "\n";
  }

  if (failures != 0) {
    std::cerr << failures << " OMPL checks failed\n";
    return 1;
  }
  std::cout << "test_ompl passed\n";
  return 0;
}
