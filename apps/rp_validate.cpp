#include "rp/adapters/kdl_kinematics.h"
#include "rp/api/scene_io.h"
#include "rp/core/collision/collision.h"
#include "rp/core/kinematics/kinematics.h"
#include "rp/planning/planner.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#ifndef RP_DEFAULT_CONFIG
#define RP_DEFAULT_CONFIG "scenes/car_door.json"
#endif

namespace {

struct CaseResult {
  int index = 0;
  double center_u = 0.0;
  double center_v = 0.0;
  bool pass = false;
  bool success = false;
  bool collision_free = false;
  bool order_valid = false;
  bool tangential_within = true;
  double coverage = 0.0;
  int samples_feasible = 0;
  int samples_total = 0;
  int runs = 0;
  double min_clearance = 0.0;
  double max_scan_joint_step = 0.0;
  double max_transfer_joint_step = 0.0;
  double max_standoff_error = 0.0;
  double max_alignment_error = 0.0;
  double total_time = 0.0;
  double tcp_path = 0.0;
  double planning_time = 0.0;
  std::vector<std::string> warnings;
};

double maxJointDelta(const rp::VecX& a, const rp::VecX& b) {
  double result = 0.0;
  for (Eigen::Index i = 0; i < a.size() && i < b.size(); ++i) {
    result = std::max(result, std::abs(a[i] - b[i]));
  }
  return result;
}

CaseResult runCase(int index, const rp::SceneConfig& config, const rp::TargetSurface& surface) {
  CaseResult report;
  report.index = index;
  report.center_u = (surface.origin - config.surface.origin).dot(config.surface.u_axis.normalized()) +
                    surface.size_u * 0.5;
  report.center_v = (surface.origin - config.surface.origin).dot(config.surface.v_axis.normalized()) +
                    surface.size_v * 0.5;

  const auto start = std::chrono::steady_clock::now();
  const rp::PlanResult plan =
      rp::Planner::plan(config.scene, surface, config.scanner, config.options);
  const auto finish = std::chrono::steady_clock::now();
  report.planning_time = std::chrono::duration<double>(finish - start).count();

  report.success = plan.success;
  report.collision_free = plan.metrics.collision_free;
  report.order_valid = plan.metrics.order_valid;
  report.coverage = plan.metrics.coverage_ratio;
  report.samples_feasible = plan.metrics.samples_feasible;
  report.samples_total = plan.metrics.samples_total;
  report.runs = plan.metrics.runs_planned;
  report.total_time = plan.metrics.total_time_s;
  report.tcp_path = plan.metrics.tcp_path_m;
  report.warnings = plan.warnings;

  rp::CollisionOptions collision_options;
  collision_options.margin = config.options.collision_margin;
  collision_options.check_ground = config.scene.has_ground;
  collision_options.ground_z = config.scene.ground_z;
  const rp::CollisionChecker checker(config.scene, collision_options);

  const rp::Vec3 normal = surface.resolvedNormal();
  const rp::Vec3 u_axis = surface.u_axis.normalized();
  const rp::Vec3 v_axis = surface.v_axis.normalized();

  report.min_clearance = std::numeric_limits<double>::max();
  for (const rp::PlannedTransition& transition : plan.transitions) {
    for (std::size_t i = 0; i < transition.waypoints.size(); ++i) {
      report.min_clearance =
          std::min(report.min_clearance, checker.checkConfiguration(transition.waypoints[i].q).min_distance);
      if (i > 0) {
        report.max_transfer_joint_step =
            std::max(report.max_transfer_joint_step,
                     maxJointDelta(transition.waypoints[i - 1].q, transition.waypoints[i].q));
      }
    }
  }

  for (const rp::PlannedRun& run : plan.runs) {
    for (std::size_t i = 0; i < run.waypoints.size(); ++i) {
      const rp::PlannedWaypoint& waypoint = run.waypoints[i];
      report.min_clearance =
          std::min(report.min_clearance, checker.checkConfiguration(waypoint.q).min_distance);
      if (i > 0) {
        report.max_scan_joint_step =
            std::max(report.max_scan_joint_step,
                     maxJointDelta(run.waypoints[i - 1].q, waypoint.q));
      }

      const double signed_distance = (waypoint.tcp - surface.origin).dot(normal);
      report.max_standoff_error =
          std::max(report.max_standoff_error, std::abs(signed_distance - config.scanner.standoff));

      const rp::Mat3 rotation = rp::Kinematics::forward(config.scene.robot, waypoint.q).tool.linear();
      const double alignment = -rotation.col(2).dot(normal);
      report.max_alignment_error =
          std::max(report.max_alignment_error, std::acos(std::min(1.0, std::max(-1.0, alignment))));

      const double u = (waypoint.tcp - surface.origin).dot(u_axis);
      const double v = (waypoint.tcp - surface.origin).dot(v_axis);
      const double tolerance = 1e-4;
      if (u < -tolerance || u > surface.size_u + tolerance || v < -tolerance ||
          v > surface.size_v + tolerance) {
        report.tangential_within = false;
      }
    }
  }

  if (plan.runs.empty()) {
    report.min_clearance = 0.0;
  }

  report.pass = report.success && report.coverage >= 0.999 && report.collision_free &&
                report.order_valid && report.tangential_within &&
                report.min_clearance + 1e-9 >= config.options.collision_margin &&
                report.max_scan_joint_step <= config.options.max_scan_joint_step + 1e-9 &&
                report.max_standoff_error <= 1e-3 && report.max_alignment_error <= 1e-3;
  return report;
}

void printUsage() {
  std::cout << "usage: rp_validate [config.json] [--patch-size U V] [--grid NU NV] "
               "[--fov WIDTH] [--spacing STEP] [--cell SIZE] [--planner NAME] "
               "[--ik internal|kdl] [--collision internal|fcl] "
               "[--out report.json] [--sweep-v] [--no-order]\n";
}

bool parseDouble(const char* text, double* value) {
  try {
    std::size_t consumed = 0;
    const std::string input(text);
    *value = std::stod(input, &consumed);
    return consumed == input.size();
  } catch (...) {
    return false;
  }
}

bool parseInt(const char* text, int* value) {
  try {
    std::size_t consumed = 0;
    const std::string input(text);
    *value = std::stoi(input, &consumed);
    return consumed == input.size();
  } catch (...) {
    return false;
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = RP_DEFAULT_CONFIG;
  std::string output_path = "validation_small_range.json";
  double patch_u = 0.12;
  double patch_v = 0.10;
  int grid_u = 3;
  int grid_v = 3;
  double fov = 0.04;
  double spacing = 0.005;
  double cell = 0.002;
  bool sweep_along_u = true;
  bool optimize_order = true;
  rp::MotionPlannerBackend motion_backend = rp::MotionPlannerBackend::InternalRRTConnect;
  rp::IKBackend ik_backend = rp::IKBackend::Internal;
  rp::CollisionBackend collision_backend = rp::CollisionBackend::Internal;

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      printUsage();
      return 0;
    }
    if (argument == "--patch-size" && i + 2 < argc) {
      if (!parseDouble(argv[++i], &patch_u) || !parseDouble(argv[++i], &patch_v)) {
        std::cerr << "invalid --patch-size\n";
        return 2;
      }
      continue;
    }
    if (argument == "--grid" && i + 2 < argc) {
      if (!parseInt(argv[++i], &grid_u) || !parseInt(argv[++i], &grid_v)) {
        std::cerr << "invalid --grid\n";
        return 2;
      }
      continue;
    }
    if (argument == "--fov" && i + 1 < argc) {
      if (!parseDouble(argv[++i], &fov)) {
        std::cerr << "invalid --fov\n";
        return 2;
      }
      continue;
    }
    if (argument == "--spacing" && i + 1 < argc) {
      if (!parseDouble(argv[++i], &spacing)) {
        std::cerr << "invalid --spacing\n";
        return 2;
      }
      continue;
    }
    if (argument == "--cell" && i + 1 < argc) {
      if (!parseDouble(argv[++i], &cell)) {
        std::cerr << "invalid --cell\n";
        return 2;
      }
      continue;
    }
    if (argument == "--out" && i + 1 < argc) {
      output_path = argv[++i];
      continue;
    }
    if (argument == "--planner" && i + 1 < argc) {
      const std::string name = argv[++i];
      if (!rp::parseMotionBackend(name, &motion_backend)) {
        std::cerr << "unknown planner: " << name << "\n";
        return 2;
      }
      continue;
    }
    if (argument == "--ik" && i + 1 < argc) {
      const std::string name = argv[++i];
      if (name == "kdl") {
        ik_backend = rp::IKBackend::Kdl;
      } else if (name == "internal") {
        ik_backend = rp::IKBackend::Internal;
      } else {
        std::cerr << "unknown ik backend: " << name << "\n";
        return 2;
      }
      continue;
    }
    if (argument == "--collision" && i + 1 < argc) {
      const std::string name = argv[++i];
      if (name == "fcl") {
        collision_backend = rp::CollisionBackend::Fcl;
      } else if (name == "internal") {
        collision_backend = rp::CollisionBackend::Internal;
      } else {
        std::cerr << "unknown collision backend: " << name << "\n";
        return 2;
      }
      continue;
    }
    if (argument == "--sweep-v") {
      sweep_along_u = false;
      continue;
    }
    if (argument == "--no-order") {
      optimize_order = false;
      continue;
    }
    if (!argument.empty() && argument[0] != '-') {
      config_path = argument;
      continue;
    }
    std::cerr << "unknown argument: " << argument << "\n";
    printUsage();
    return 2;
  }

  if (grid_u < 1 || grid_v < 1 || patch_u <= 0.0 || patch_v <= 0.0 || fov <= 0.0 ||
      spacing <= 0.0 || cell <= 0.0) {
    std::cerr << "invalid patch, grid, or scanner parameters\n";
    return 2;
  }

  try {
    rp::SceneConfig config = rp::loadSceneConfig(config_path);
    if (config.task != rp::TaskType::SurfaceCoverage) {
      std::cerr << "rp_validate only supports surface_coverage tasks\n";
      return 2;
    }
    config.scanner.fov_width = fov;
    config.scanner.sample_spacing = spacing;
    config.options.coverage_cell = cell;
    config.options.sweep_along_u = sweep_along_u;
    config.options.optimize_order = optimize_order;
    config.options.require_full_coverage = true;
    config.options.motion_backend = motion_backend;
    config.options.ik_backend = ik_backend;
    config.options.collision_backend = collision_backend;
    if (ik_backend == rp::IKBackend::Kdl && !rp::isKdlAvailable()) {
      std::cerr << "KDL IK backend is not available in this build\n";
      return 2;
    }
    if (collision_backend == rp::CollisionBackend::Fcl && !rp::isFclAvailable()) {
      std::cerr << "FCL collision backend is not available in this build\n";
      return 2;
    }
    if (rp::isOmplBackend(motion_backend) && !rp::isOmplAvailable()) {
      std::cerr << "planner " << rp::motionBackendName(motion_backend)
                << " requires an OMPL-enabled build\n";
      return 2;
    }

    const double usable_u = std::min(patch_u, config.surface.size_u);
    const double usable_v = std::min(patch_v, config.surface.size_v);

    std::vector<CaseResult> cases;
    for (int iv = 0; iv < grid_v; ++iv) {
      for (int iu = 0; iu < grid_u; ++iu) {
        const double center_u =
            (grid_u == 1) ? config.surface.size_u * 0.5
                          : usable_u * 0.5 + (config.surface.size_u - usable_u) * iu / (grid_u - 1);
        const double center_v =
            (grid_v == 1) ? config.surface.size_v * 0.5
                          : usable_v * 0.5 + (config.surface.size_v - usable_v) * iv / (grid_v - 1);
        const rp::TargetSurface patch =
            rp::makeSubSurface(config.surface, center_u, center_v, usable_u, usable_v);
        cases.push_back(runCase(static_cast<int>(cases.size()), config, patch));
      }
    }

    int passes = 0;
    double coverage_sum = 0.0;
    double min_clearance = std::numeric_limits<double>::max();
    double max_scan_step = 0.0;
    double max_transfer_step = 0.0;
    double max_standoff_error = 0.0;
    double max_alignment_error = 0.0;
    double planning_time_sum = 0.0;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "config: " << config_path << "\n";
    std::cout << "planner: " << rp::motionBackendName(motion_backend)
              << " (ompl_available=" << (rp::isOmplAvailable() ? "true" : "false") << ")\n";
    std::cout << "ik: " << rp::ikBackendName(ik_backend)
              << " (kdl_available=" << (rp::isKdlAvailable() ? "true" : "false") << ")\n";
    std::cout << "collision: " << rp::collisionBackendName(collision_backend)
              << " (fcl_available=" << (rp::isFclAvailable() ? "true" : "false") << ")\n";
    std::cout << "patch_size: " << usable_u << " x " << usable_v << " m, grid: " << grid_u << " x "
              << grid_v << "\n";
    std::cout << "case  center(u,v)          coverage  feasible  min_clear  scan_step  transfer_step  "
                 "time_s  pass\n";
    for (const CaseResult& result : cases) {
      passes += result.pass ? 1 : 0;
      coverage_sum += result.coverage;
      min_clearance = std::min(min_clearance, result.min_clearance);
      max_scan_step = std::max(max_scan_step, result.max_scan_joint_step);
      max_transfer_step = std::max(max_transfer_step, result.max_transfer_joint_step);
      max_standoff_error = std::max(max_standoff_error, result.max_standoff_error);
      max_alignment_error = std::max(max_alignment_error, result.max_alignment_error);
      planning_time_sum += result.planning_time;

      std::ostringstream center;
      center << "(" << result.center_u << "," << result.center_v << ")";
      std::cout << std::setw(4) << result.index << "  " << std::left << std::setw(20) << center.str()
                << std::right << "  " << std::setw(8) << result.coverage << "  " << std::setw(8)
                << (std::to_string(result.samples_feasible) + "/" +
                    std::to_string(result.samples_total))
                << "  " << std::setw(9) << result.min_clearance << "  " << std::setw(9)
                << result.max_scan_joint_step << "  " << std::setw(13)
                << result.max_transfer_joint_step << "  " << std::setw(6) << result.total_time
                << "  " << (result.pass ? "yes" : "no") << "\n";
      for (const std::string& warning : result.warnings) {
        std::cout << "      warning: " << warning << "\n";
      }
    }

    const int count = static_cast<int>(cases.size());
    nlohmann::json report;
    report["config"] = config_path;
    report["planner"] = rp::motionBackendName(motion_backend);
    report["ik_backend"] = rp::ikBackendName(ik_backend);
    report["collision_backend"] = rp::collisionBackendName(collision_backend);
    report["ompl_available"] = rp::isOmplAvailable();
    report["kdl_available"] = rp::isKdlAvailable();
    report["fcl_available"] = rp::isFclAvailable();
    report["patch_size"] = {usable_u, usable_v};
    report["grid"] = {grid_u, grid_v};
    report["scanner"] = {
        {"fov_width", fov},
        {"sample_spacing", spacing},
        {"coverage_cell", cell},
        {"sweep_along_u", sweep_along_u},
        {"optimize_order", optimize_order},
    };
    report["summary"] = {
        {"cases", count},
        {"passes", passes},
        {"pass_rate", count > 0 ? static_cast<double>(passes) / count : 0.0},
        {"mean_coverage", count > 0 ? coverage_sum / count : 0.0},
        {"min_clearance_m", min_clearance},
        {"max_scan_joint_step_rad", max_scan_step},
        {"max_transfer_joint_step_rad", max_transfer_step},
        {"max_standoff_error_m", max_standoff_error},
        {"max_alignment_error_rad", max_alignment_error},
        {"planning_time_s", planning_time_sum},
    };

    nlohmann::json case_array = nlohmann::json::array();
    for (const CaseResult& result : cases) {
      nlohmann::json entry;
      entry["index"] = result.index;
      entry["center_u"] = result.center_u;
      entry["center_v"] = result.center_v;
      entry["pass"] = result.pass;
      entry["success"] = result.success;
      entry["coverage"] = result.coverage;
      entry["samples_feasible"] = result.samples_feasible;
      entry["samples_total"] = result.samples_total;
      entry["runs"] = result.runs;
      entry["collision_free"] = result.collision_free;
      entry["order_valid"] = result.order_valid;
      entry["tangential_within"] = result.tangential_within;
      entry["min_clearance_m"] = result.min_clearance;
      entry["max_scan_joint_step_rad"] = result.max_scan_joint_step;
      entry["max_transfer_joint_step_rad"] = result.max_transfer_joint_step;
      entry["max_standoff_error_m"] = result.max_standoff_error;
      entry["max_alignment_error_rad"] = result.max_alignment_error;
      entry["total_time_s"] = result.total_time;
      entry["tcp_path_m"] = result.tcp_path;
      entry["planning_time_s"] = result.planning_time;
      entry["warnings"] = result.warnings;
      case_array.push_back(std::move(entry));
    }
    report["cases"] = std::move(case_array);

    std::ofstream stream(output_path, std::ios::binary);
    if (!stream) {
      std::cerr << "failed to write " << output_path << "\n";
      return 1;
    }
    stream << report.dump(2);

    std::cout << "summary: " << passes << "/" << count << " passed"
              << ", mean_coverage=" << (count > 0 ? coverage_sum / count : 0.0)
              << ", min_clearance=" << min_clearance << ", max_scan_step=" << max_scan_step
              << ", max_standoff_error=" << max_standoff_error
              << ", max_alignment_error=" << max_alignment_error
              << ", planning_time=" << planning_time_sum << "s\n";
    std::cout << "report: " << output_path << "\n";

    return passes == count ? 0 : 1;
  } catch (const std::exception& exception) {
    std::cerr << "error: " << exception.what() << "\n";
    return 1;
  }
}
