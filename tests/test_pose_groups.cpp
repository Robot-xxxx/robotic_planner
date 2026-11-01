#include "rp/api/scene_io.h"
#include "rp/planning/planner.h"

#include <nlohmann/json.hpp>

#include <algorithm>
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

void validateGroups(const rp::SceneConfig& config, const rp::PlanResult& plan,
                    const std::string& label) {
  check(plan.success, label + ": plan succeeds: " + plan.message);
  check(plan.task_type == "pose_groups", label + ": task type");
  check(plan.metrics.pose_groups_total == static_cast<int>(config.pose_groups.size()),
        label + ": group count");
  check(plan.metrics.pose_groups_planned == static_cast<int>(config.pose_groups.size()),
        label + ": all groups planned");
  check(plan.pose_groups.size() == config.pose_groups.size(), label + ": group summaries");
  check(plan.pose_visits.size() == plan.metrics.poses_reachable, label + ": visit count");
  check(plan.metrics.collision_free, label + ": collision free");
  check(plan.metrics.order_valid, label + ": order valid");

  int previous_group = -1;
  std::vector<std::vector<int>> visited_indices(config.pose_groups.size());
  std::vector<int> group_seen(config.pose_groups.size(), 0);
  for (const rp::PlannedPoseVisit& visit : plan.pose_visits) {
    check(visit.group_index >= 0 &&
              visit.group_index < static_cast<int>(config.pose_groups.size()),
          label + ": valid group index");
    check(visit.group_index >= previous_group, label + ": groups stay in input order");
    previous_group = visit.group_index;
    visited_indices[static_cast<std::size_t>(visit.group_index)].push_back(
        visit.group_pose_index);
    ++group_seen[static_cast<std::size_t>(visit.group_index)];
  }

  for (std::size_t g = 0; g < config.pose_groups.size(); ++g) {
    check(group_seen[g] == static_cast<int>(config.pose_groups[g].poses.size()),
          label + ": all poses in group " + std::to_string(g) + " visited");
    check(plan.pose_groups[g].poses_visited == group_seen[g],
          label + ": group summary matches visits");
    if (!config.pose_groups[g].optimize_order) {
      for (std::size_t i = 1; i < visited_indices[g].size(); ++i) {
        check(visited_indices[g][i] > visited_indices[g][i - 1],
              label + ": non-optimized group preserves input order");
      }
    }
  }

  const std::string json = rp::planResultToJson(config, plan);
  const auto parsed = nlohmann::json::parse(json);
  check(parsed.value("task", "") == "pose_groups", label + ": JSON task type");
  check(parsed.contains("pose_groups") && parsed["pose_groups"].size() == 3,
        label + ": JSON group summaries");
  check(parsed["pose_visits"].front().contains("group_index"),
        label + ": JSON visit group index");
}

}  // namespace

int main() {
  const std::string path = std::string(RP_SOURCE_DIR) + "/scenes/pose_groups.json";
  rp::SceneConfig config = rp::loadSceneConfig(path);
  check(config.task == rp::TaskType::PoseGroups, "config parses pose groups task");
  check(config.pose_groups.size() == 3, "config contains three groups");
  check(config.pose_groups[1].optimize_order == false, "group optimize flag parsed");

  rp::PlanningOptions options = config.options;
  options.motion_backend = rp::MotionPlannerBackend::InternalRRTConnect;
  options.optimize_order = true;
  const rp::PlanResult plan =
      rp::Planner::planPoseGroups(config.scene, config.pose_groups, options);
  validateGroups(config, plan, "optimized_groups");

  rp::PlanningOptions preserve = options;
  preserve.optimize_order = false;
  const rp::PlanResult preserve_plan =
      rp::Planner::planPoseGroups(config.scene, config.pose_groups, preserve);
  validateGroups(config, preserve_plan, "preserved_groups");
  std::vector<int> expected_group;
  std::vector<int> expected_pose;
  for (std::size_t g = 0; g < config.pose_groups.size(); ++g) {
    for (std::size_t p = 0; p < config.pose_groups[g].poses.size(); ++p) {
      expected_group.push_back(static_cast<int>(g));
      expected_pose.push_back(static_cast<int>(p));
    }
  }
  for (std::size_t i = 0; i < preserve_plan.pose_visits.size(); ++i) {
    check(preserve_plan.pose_visits[i].group_index == expected_group[i],
          "global order preservation keeps group order");
    check(preserve_plan.pose_visits[i].group_pose_index == expected_pose[i],
          "global order preservation keeps pose order");
  }

  rp::PlanningOptions with_return = options;
  with_return.return_home = true;
  const rp::PlanResult return_plan =
      rp::Planner::planPoseGroups(config.scene, config.pose_groups, with_return);
  check(return_plan.success, "group plan with return home succeeds");
  check(return_plan.transitions.size() == return_plan.pose_visits.size() + 1,
        "return home adds one transfer");

  if (failures != 0) {
    std::cerr << failures << " pose group checks failed\n";
    return 1;
  }
  std::cout << "test_pose_groups passed\n";
  return 0;
}
