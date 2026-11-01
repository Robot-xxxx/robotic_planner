#include "rp/adapters/kdl_kinematics.h"
#include "rp/adapters/tesseract_planner.h"
#include "rp/api/scene_io.h"
#include "rp/planning/planner.h"

#include <chrono>
#include <iostream>
#include <string>

#ifndef RP_DEFAULT_CONFIG
#define RP_DEFAULT_CONFIG "scenes/car_door.json"
#endif

namespace {

void printUsage() {
  std::cout << "usage: rp_cli [config.json] [--out plan.json] [--planner NAME] [--ik NAME] "
               "[--collision NAME] [--no-order] [--require-full] [--sweep-v] [--json]\n"
               "  config task: surface_coverage, pose_sequence, or pose_groups (auto-detected)\n"
               "  planner names: internal_rrtconnect, ompl_rrtconnect, ompl_rrtstar, "
               "ompl_bitstar, ompl_prm, ompl_est, ompl_kpiece1, tesseract_ompl\n"
               "  ik names: internal, kdl; collision names: internal, fcl\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path = RP_DEFAULT_CONFIG;
  std::string output_path = "plan.json";
  bool json_only = false;

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      printUsage();
      return 0;
    }
    if (argument == "--out") {
      if (i + 1 >= argc) {
        std::cerr << "--out requires a path\n";
        return 2;
      }
      output_path = argv[++i];
      continue;
    }
    if (argument == "--planner" || argument == "--ik" || argument == "--collision") {
      if (i + 1 >= argc) {
        std::cerr << argument << " requires a name\n";
        return 2;
      }
      ++i;
      continue;
    }
    if (argument == "--no-order") {
      continue;
    }
    if (argument == "--json") {
      json_only = true;
      continue;
    }
    if (argument == "--require-full") {
      continue;
    }
    if (argument == "--sweep-v") {
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

  try {
    rp::SceneConfig config = rp::loadSceneConfig(config_path);
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--no-order") {
        config.options.optimize_order = false;
      } else if (argument == "--require-full") {
        config.options.require_full_coverage = true;
      } else if (argument == "--sweep-v") {
        config.options.sweep_along_u = false;
      } else if (argument == "--planner" && i + 1 < argc) {
        const std::string name = argv[++i];
        if (!rp::parseMotionBackend(name, &config.options.motion_backend)) {
          std::cerr << "unknown planner: " << name << "\n";
          return 2;
        }
      } else if (argument == "--ik" && i + 1 < argc) {
        const std::string name = argv[++i];
        if (name == "kdl") {
          config.options.ik_backend = rp::IKBackend::Kdl;
        } else if (name == "internal") {
          config.options.ik_backend = rp::IKBackend::Internal;
        } else {
          std::cerr << "unknown ik backend: " << name << "\n";
          return 2;
        }
      } else if (argument == "--collision" && i + 1 < argc) {
        const std::string name = argv[++i];
        if (name == "fcl") {
          config.options.collision_backend = rp::CollisionBackend::Fcl;
        } else if (name == "internal") {
          config.options.collision_backend = rp::CollisionBackend::Internal;
        } else {
          std::cerr << "unknown collision backend: " << name << "\n";
          return 2;
        }
      }
    }

    if (rp::isOmplBackend(config.options.motion_backend) && !rp::isOmplAvailable()) {
      std::cerr << "planner " << rp::motionBackendName(config.options.motion_backend)
                << " requires an OMPL-enabled build\n";
      return 2;
    }
    if (rp::isTesseractBackend(config.options.motion_backend) && !rp::isTesseractAvailable()) {
      std::cerr << "planner " << rp::motionBackendName(config.options.motion_backend)
                << " requires a Tesseract-enabled build\n";
      return 2;
    }

    const auto start = std::chrono::steady_clock::now();
    const rp::PlanResult result = rp::planScene(config);
    const auto finish = std::chrono::steady_clock::now();
    const double planning_seconds =
        std::chrono::duration<double>(finish - start).count();

    if (!rp::savePlanJson(output_path, config, result)) {
      std::cerr << "failed to write " << output_path << "\n";
      return 1;
    }

    if (json_only) {
      std::cout << rp::planResultToJson(config, result) << "\n";
      return result.success ? 0 : 1;
    }

    const rp::PlanMetrics& metrics = result.metrics;
    std::cout << "config: " << config_path << "\n";
    std::cout << "task: " << result.task_type << "\n";
    std::cout << "planner: " << rp::motionBackendName(config.options.motion_backend)
              << " (ompl_available=" << (rp::isOmplAvailable() ? "true" : "false")
              << ", tesseract_available=" << (rp::isTesseractAvailable() ? "true" : "false")
              << ")\n";
    std::cout << "ik: " << rp::ikBackendName(config.options.ik_backend)
              << " (kdl_available=" << (rp::isKdlAvailable() ? "true" : "false") << ")\n";
    std::cout << "collision: " << rp::collisionBackendName(config.options.collision_backend)
              << " (fcl_available=" << (rp::isFclAvailable() ? "true" : "false") << ")\n";
    std::cout << "success: " << (result.success ? "true" : "false") << "\n";
    std::cout << "message: " << result.message << "\n";
    std::cout << "planning_time_s: " << planning_seconds << "\n";
    std::cout << "total_time_s: " << metrics.total_time_s << "\n";
    std::cout << "scan_time_s: " << metrics.scan_time_s << "\n";
    std::cout << "transfer_time_s: " << metrics.transfer_time_s << "\n";
    std::cout << "tcp_path_m: " << metrics.tcp_path_m << "\n";
    std::cout << "agv_distance_m: " << metrics.agv_distance_m << "\n";
    std::cout << "external_revolute_rad: " << metrics.external_revolute_rad << "\n";
    std::cout << "external_prismatic_m: " << metrics.external_prismatic_m << "\n";
    std::cout << "coverage_ratio: " << metrics.coverage_ratio << "\n";
    std::cout << "covered_area_m2: " << metrics.covered_area_m2 << "\n";
    std::cout << "target_area_m2: " << metrics.target_area_m2 << "\n";
    std::cout << "strips_generated: " << metrics.strips_generated << "\n";
    std::cout << "runs_planned: " << metrics.runs_planned << "\n";
    std::cout << "samples: " << metrics.samples_feasible << "/" << metrics.samples_total << "\n";
    if (result.task_type == "pose_sequence" || result.task_type == "pose_groups") {
      std::cout << "poses: " << metrics.poses_reachable << "/" << metrics.poses_total << "\n";
      if (result.task_type == "pose_groups") {
        std::cout << "groups: " << metrics.pose_groups_planned << "/"
                  << metrics.pose_groups_total << "\n";
      }
      std::cout << "dwell_time_s: " << metrics.dwell_time_s << "\n";
    }
    std::cout << "collision_free: " << (metrics.collision_free ? "true" : "false") << "\n";
    std::cout << "order_valid: " << (metrics.order_valid ? "true" : "false") << "\n";
    std::cout << "full_coverage: " << (metrics.full_coverage ? "true" : "false") << "\n";
    std::cout << "plan_json: " << output_path << "\n";
    for (const std::string& warning : result.warnings) {
      std::cout << "warning: " << warning << "\n";
    }

    return result.success ? 0 : 1;
  } catch (const std::exception& exception) {
    std::cerr << "error: " << exception.what() << "\n";
    return 1;
  }
}
