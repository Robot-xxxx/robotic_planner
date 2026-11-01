#include "rp/api/scene_io.h"
#include "rp/core/collision/collision.h"
#include "rp/planning/planner.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <iostream>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    ++failures;
  }
}

}  // namespace

int main() {
  const rp::SceneConfig config = rp::loadSceneConfig(RP_DEFAULT_CONFIG);
  const rp::PlanResult plan =
      rp::Planner::plan(config.scene, config.surface, config.scanner, config.options);

  check(plan.success, "demo plan succeeds: " + plan.message);
  check(plan.metrics.coverage_ratio > 0.99, "demo plan reaches near-full coverage");
  check(plan.metrics.runs_planned > 0, "demo plan contains scan runs");
  check(!plan.runs.empty(), "demo plan exposes run trajectories");
  check(!plan.transitions.empty(), "demo plan exposes transfer trajectories");
  check(plan.metrics.total_time_s > 0.0, "total time is positive");
  check(plan.metrics.tcp_path_m > 0.0, "TCP path is positive");
  check(plan.metrics.collision_free, "demo plan is collision free");
  check(plan.metrics.order_valid, "demo plan order is valid");
  check(plan.metrics.samples_feasible > 0, "some samples are feasible");

  rp::CollisionOptions options;
  options.margin = config.options.collision_margin;
  options.check_ground = config.scene.has_ground;
  options.ground_z = config.scene.ground_z;
  const rp::CollisionChecker checker(config.scene, options);

  for (const rp::PlannedRun& run : plan.runs) {
    check(run.waypoints.size() >= 2, "run has at least two waypoints");
    check(run.end_time >= run.start_time, "run times are ordered");
    for (std::size_t i = 0; i < run.waypoints.size(); ++i) {
      check(!checker.checkConfiguration(run.waypoints[i].q).collision,
            "run waypoint is collision free");
      if (i > 0) {
        check(run.waypoints[i].time >= run.waypoints[i - 1].time, "waypoint times are ordered");
      }
    }
  }

  for (const rp::PlannedTransition& transition : plan.transitions) {
    check(transition.duration >= 0.0, "transition duration is non-negative");
    for (std::size_t i = 0; i < transition.waypoints.size(); ++i) {
      check(!checker.checkConfiguration(transition.waypoints[i].q).collision,
            "transition waypoint is collision free");
    }
  }

  const std::string json = rp::planResultToJson(config, plan);
  const auto parsed = nlohmann::json::parse(json);
  check(parsed.contains("metrics"), "plan JSON contains metrics");
  check(parsed["metrics"]["coverage_ratio"].get<double>() > 0.99, "JSON coverage matches");
  check(parsed.contains("runs") && !parsed["runs"].empty(), "JSON contains runs");

  rp::PlanningOptions no_order = config.options;
  no_order.optimize_order = false;
  const rp::PlanResult unoptimized =
      rp::Planner::plan(config.scene, config.surface, config.scanner, no_order);
  check(unoptimized.success, "plan without order optimization succeeds");
  check(unoptimized.metrics.coverage_ratio > 0.99, "unoptimized plan keeps coverage");

  if (failures != 0) {
    std::cerr << failures << " planner checks failed\n";
    return 1;
  }
  std::cout << "test_planner passed\n";
  return 0;
}
