#pragma once

#include "rp/planning/planner.h"

#include <string>
#include <vector>

namespace rp {

enum class TaskType { SurfaceCoverage, PoseSequence, PoseGroups };

struct SceneConfig {
  TaskType task = TaskType::SurfaceCoverage;
  SceneModel scene;
  TargetSurface surface;
  ScannerParams scanner;
  PlanningOptions options;
  std::vector<PoseTarget> poses;
  std::vector<PoseGroup> pose_groups;
  double default_pose_dwell = 0.5;
};

SceneConfig parseSceneConfig(const std::string& json_text);
SceneConfig loadSceneConfig(const std::string& path);

PlanResult planScene(const SceneConfig& config);

std::string planResultToJson(const SceneConfig& config, const PlanResult& result);
bool savePlanJson(const std::string& path, const SceneConfig& config, const PlanResult& result);

}  // namespace rp
