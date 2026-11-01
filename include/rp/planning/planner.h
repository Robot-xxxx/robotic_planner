#pragma once

#include "rp/adapters/motion_planner.h"
#include "rp/core/collision/collision.h"
#include "rp/core/scene_model/scene.h"
#include "rp/planning/coverage/coverage.h"

#include <string>
#include <vector>

namespace rp {

enum class IKBackend { Internal, Kdl };

inline const char* ikBackendName(IKBackend backend) {
  return backend == IKBackend::Kdl ? "kdl" : "internal";
}

struct PlanningOptions {
  IKBackend ik_backend = IKBackend::Internal;
  CollisionBackend collision_backend = CollisionBackend::Internal;
  double collision_margin = 0.005;
  double collision_step = 0.05;
  double coverage_cell = 0.01;
  int ik_max_iterations = 180;
  int ik_seed_attempts = 8;
  double max_scan_joint_step = 0.9;
  bool optimize_order = true;
  bool require_full_coverage = false;
  bool sweep_along_u = true;
  bool verbose = false;
  MotionPlannerBackend motion_backend = MotionPlannerBackend::InternalRRTConnect;
  double ompl_time_limit = 0.5;
  double ompl_range = 0.0;
  unsigned int ompl_seed = 1;
  bool ompl_simplify = true;
  bool return_home = false;
  bool require_all_poses = true;
};

struct PoseTarget {
  std::string name;
  Isometry pose = Isometry::Identity();
  double dwell_time = 0.5;
};

struct PoseGroup {
  std::string name;
  std::vector<PoseTarget> poses;
  bool optimize_order = true;
};

struct PlannedWaypoint {
  double time = 0.0;
  VecX q;
  Vec3 tcp = Vec3::Zero();
};

struct PlannedRun {
  int id = 0;
  int strip_id = 0;
  bool reversed = false;
  double start_time = 0.0;
  double end_time = 0.0;
  double duration = 0.0;
  double length_m = 0.0;
  std::vector<PlannedWaypoint> waypoints;
};

struct PlannedTransition {
  int from_run = -1;
  int to_run = -1;
  int from_pose = -1;
  int to_pose = -1;
  double start_time = 0.0;
  double end_time = 0.0;
  double duration = 0.0;
  double length_m = 0.0;
  std::vector<PlannedWaypoint> waypoints;
};

struct PlannedPoseVisit {
  std::string name;
  int input_index = -1;
  int order = -1;
  int group_index = -1;
  int group_pose_index = -1;
  std::string group_name;
  VecX q;
  Vec3 tcp = Vec3::Zero();
  double arrival_time = 0.0;
  double dwell_time = 0.0;
};

struct PlannedPoseGroup {
  std::string name;
  int input_index = -1;
  int poses_total = 0;
  int poses_visited = 0;
  bool optimize_order = true;
  double start_time = 0.0;
  double end_time = 0.0;
};

struct PlanMetrics {
  double total_time_s = 0.0;
  double scan_time_s = 0.0;
  double transfer_time_s = 0.0;
  double dwell_time_s = 0.0;
  double tcp_path_m = 0.0;
  double agv_distance_m = 0.0;
  double external_revolute_rad = 0.0;
  double external_prismatic_m = 0.0;
  double target_area_m2 = 0.0;
  double covered_area_m2 = 0.0;
  double coverage_ratio = 0.0;
  int strips_generated = 0;
  int runs_planned = 0;
  int samples_total = 0;
  int samples_feasible = 0;
  int poses_total = 0;
  int poses_reachable = 0;
  int pose_groups_total = 0;
  int pose_groups_planned = 0;
  bool collision_free = true;
  bool order_valid = true;
  bool full_coverage = false;
};

struct PlanResult {
  bool success = false;
  std::string task_type = "surface_coverage";
  std::string message;
  PlanMetrics metrics;
  std::vector<PlannedRun> runs;
  std::vector<PlannedPoseVisit> pose_visits;
  std::vector<PlannedPoseGroup> pose_groups;
  std::vector<PlannedTransition> transitions;
  std::vector<std::string> warnings;
};

class Planner {
 public:
  static PlanResult plan(const SceneModel& scene, const TargetSurface& surface,
                         const ScannerParams& scanner, const PlanningOptions& options);

  static PlanResult planPoseSequence(const SceneModel& scene,
                                     const std::vector<PoseTarget>& poses,
                                     const PlanningOptions& options);

  static PlanResult planPoseGroups(const SceneModel& scene,
                                   const std::vector<PoseGroup>& groups,
                                   const PlanningOptions& options);
};

}  // namespace rp
