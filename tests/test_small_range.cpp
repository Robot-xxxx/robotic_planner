#include "rp/api/scene_io.h"
#include "rp/core/collision/collision.h"
#include "rp/core/kinematics/kinematics.h"
#include "rp/planning/coverage/coverage.h"
#include "rp/planning/planner.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    ++failures;
  }
}

double maxJointDelta(const rp::VecX& a, const rp::VecX& b) {
  double result = 0.0;
  for (Eigen::Index i = 0; i < a.size() && i < b.size(); ++i) {
    result = std::max(result, std::abs(a[i] - b[i]));
  }
  return result;
}

void validateCase(const rp::SceneConfig& config, const rp::TargetSurface& surface,
                  const std::string& label) {
  const rp::PlanResult plan =
      rp::Planner::plan(config.scene, surface, config.scanner, config.options);
  check(plan.success, label + ": planning succeeds: " + plan.message);
  check(plan.metrics.coverage_ratio >= 0.999, label + ": full coverage");
  check(plan.metrics.samples_feasible == plan.metrics.samples_total,
        label + ": all samples are feasible");
  check(plan.metrics.collision_free, label + ": collision free");
  check(plan.metrics.order_valid, label + ": order valid");
  check(plan.metrics.runs_planned > 0, label + ": runs planned");

  rp::CollisionOptions collision_options;
  collision_options.margin = config.options.collision_margin;
  collision_options.check_ground = config.scene.has_ground;
  collision_options.ground_z = config.scene.ground_z;
  const rp::CollisionChecker checker(config.scene, collision_options);

  const rp::Vec3 normal = surface.resolvedNormal();
  const rp::Vec3 u_axis = surface.u_axis.normalized();
  const rp::Vec3 v_axis = surface.v_axis.normalized();
  double min_clearance = std::numeric_limits<double>::max();
  double max_scan_step = 0.0;
  double max_standoff_error = 0.0;
  double max_alignment_error = 0.0;

  for (const rp::PlannedRun& run : plan.runs) {
    for (std::size_t i = 0; i < run.waypoints.size(); ++i) {
      const rp::PlannedWaypoint& waypoint = run.waypoints[i];
      min_clearance =
          std::min(min_clearance, checker.checkConfiguration(waypoint.q).min_distance);
      if (i > 0) {
        max_scan_step =
            std::max(max_scan_step, maxJointDelta(run.waypoints[i - 1].q, waypoint.q));
      }

      max_standoff_error = std::max(
          max_standoff_error,
          std::abs((waypoint.tcp - surface.origin).dot(normal) - config.scanner.standoff));
      const rp::Mat3 rotation = rp::Kinematics::forward(config.scene.robot, waypoint.q).tool.linear();
      const double alignment = -rotation.col(2).dot(normal);
      max_alignment_error =
          std::max(max_alignment_error, std::acos(std::min(1.0, std::max(-1.0, alignment))));

      const double u = (waypoint.tcp - surface.origin).dot(u_axis);
      const double v = (waypoint.tcp - surface.origin).dot(v_axis);
      check(u >= -1e-4 && u <= surface.size_u + 1e-4, label + ": TCP stays inside patch in u");
      check(v >= -1e-4 && v <= surface.size_v + 1e-4, label + ": TCP stays inside patch in v");
    }
  }

  check(min_clearance + 1e-9 >= config.options.collision_margin,
        label + ": minimum clearance respects margin");
  check(max_scan_step <= config.options.max_scan_joint_step + 1e-9,
        label + ": scan joint continuity");
  check(max_standoff_error <= 1e-3, label + ": working distance accuracy");
  check(max_alignment_error <= 1e-3, label + ": tool normal alignment accuracy");
}

}  // namespace

int main() {
  const rp::SceneConfig small = rp::loadSceneConfig(std::string(RP_SOURCE_DIR) + "/scenes/small_patch.json");
  validateCase(small, small.surface, "small_patch");

  rp::SceneConfig config = rp::loadSceneConfig(RP_DEFAULT_CONFIG);
  config.scanner.fov_width = 0.04;
  config.scanner.sample_spacing = 0.005;
  config.options.coverage_cell = 0.002;
  config.options.collision_step = 0.03;
  config.options.require_full_coverage = true;

  const double patch_u = 0.12;
  const double patch_v = 0.10;
  const int grid = 3;
  for (int iv = 0; iv < grid; ++iv) {
    for (int iu = 0; iu < grid; ++iu) {
      const double center_u =
          patch_u * 0.5 + (config.surface.size_u - patch_u) * iu / (grid - 1);
      const double center_v =
          patch_v * 0.5 + (config.surface.size_v - patch_v) * iv / (grid - 1);
      const rp::TargetSurface patch =
          rp::makeSubSurface(config.surface, center_u, center_v, patch_u, patch_v);
      validateCase(config, patch,
                   "grid[" + std::to_string(iu) + "," + std::to_string(iv) + "]");
    }
  }

  if (failures != 0) {
    std::cerr << failures << " small-range checks failed\n";
    return 1;
  }
  std::cout << "test_small_range passed\n";
  return 0;
}
