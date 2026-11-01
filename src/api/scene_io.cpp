#include "rp/api/scene_io.h"

#include "rp/adapters/kdl_kinematics.h"
#include "rp/adapters/tesseract_planner.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rp {

namespace {

using json = nlohmann::json;

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

Vec3 readVec3(const json& object, const char* key, const Vec3& fallback) {
  if (!object.contains(key)) {
    return fallback;
  }
  const auto values = object.at(key).get<std::vector<double>>();
  if (values.size() < 3) {
    throw std::runtime_error(std::string("field '") + key + "' must have three components");
  }
  return Vec3(values[0], values[1], values[2]);
}

Isometry readPose(const json& object) {
  if (object.is_null() || !object.is_object()) {
    return Isometry::Identity();
  }
  const Vec3 position = readVec3(object, "position", Vec3::Zero());
  Vec3 rpy = Vec3::Zero();
  if (object.contains("rpy_deg")) {
    rpy = readVec3(object, "rpy_deg", Vec3::Zero());
  }
  return makeIsometry(position, rotationFromRpyDegrees(rpy.x(), rpy.y(), rpy.z()));
}

Geometry readGeometry(const json& object) {
  Geometry geometry;
  if (!object.is_object()) {
    return geometry;
  }
  const std::string type = object.value("type", "none");
  if (type == "none" || type.empty()) {
    geometry.type = Geometry::Type::None;
    return geometry;
  }
  if (type == "sphere") {
    geometry.type = Geometry::Type::Sphere;
    geometry.center = readVec3(object, "center", Vec3::Zero());
    geometry.radius = object.value("radius", 0.05);
  } else if (type == "capsule") {
    geometry.type = Geometry::Type::Capsule;
    geometry.a = readVec3(object, "a", Vec3::Zero());
    geometry.b = readVec3(object, "b", Vec3(0.0, 0.0, 0.1));
    geometry.radius = object.value("radius", 0.05);
  } else if (type == "box") {
    geometry.type = Geometry::Type::Box;
    geometry.center = readVec3(object, "center", Vec3::Zero());
    if (object.contains("half_extents")) {
      geometry.half_extents = readVec3(object, "half_extents", Vec3::Ones() * 0.5);
    } else if (object.contains("size")) {
      geometry.half_extents = readVec3(object, "size", Vec3::Ones()) * 0.5;
    }
  } else if (type == "mesh") {
    geometry.type = Geometry::Type::Mesh;
    if (object.contains("vertices")) {
      const json& vertices = object.at("vertices");
      if (!vertices.empty() && vertices.front().is_number()) {
        const auto flat = vertices.get<std::vector<double>>();
        if (flat.size() % 3 != 0) {
          throw std::runtime_error("mesh vertices flat array length must be divisible by three");
        }
        for (std::size_t i = 0; i + 2 < flat.size(); i += 3) {
          geometry.vertices.emplace_back(flat[i], flat[i + 1], flat[i + 2]);
        }
      } else {
        for (const json& vertex : vertices) {
          const auto values = vertex.get<std::vector<double>>();
          if (values.size() < 3) {
            throw std::runtime_error("mesh vertex must have three components");
          }
          geometry.vertices.emplace_back(values[0], values[1], values[2]);
        }
      }
    }
    if (object.contains("triangles")) {
      for (const json& triangle : object.at("triangles")) {
        const auto indices = triangle.get<std::vector<int>>();
        if (indices.size() != 3) {
          throw std::runtime_error("mesh triangle must have three indices");
        }
        geometry.triangles.push_back({indices[0], indices[1], indices[2]});
      }
    }
  } else {
    throw std::runtime_error("unknown geometry type: " + type);
  }

  if (object.contains("local")) {
    geometry.local = readPose(object.at("local"));
  }
  return geometry;
}

Joint readJoint(const json& object) {
  Joint joint;
  joint.name = object.value("name", std::string("joint"));
  const std::string type = object.value("type", "revolute");
  if (type == "prismatic") {
    joint.type = JointType::Prismatic;
  } else if (type == "revolute") {
    joint.type = JointType::Revolute;
  } else {
    throw std::runtime_error("joint '" + joint.name + "' has unsupported type: " + type);
  }
  joint.axis = readVec3(object, "axis", Vec3::UnitZ());
  if (object.contains("origin")) {
    joint.origin = readPose(object.at("origin"));
  }
  if (object.contains("limits")) {
    const auto limits = object.at("limits").get<std::vector<double>>();
    if (limits.size() != 2) {
      throw std::runtime_error("joint '" + joint.name + "' limits must be [lower, upper]");
    }
    joint.lower = limits[0];
    joint.upper = limits[1];
  }
  joint.home = object.value("home", 0.0);
  joint.velocity_limit = object.value("velocity_limit", joint.velocity_limit);
  joint.acceleration_limit = object.value("acceleration_limit", joint.acceleration_limit);
  joint.external = object.value("external", false);
  return joint;
}

SerialChain readChain(const json& object) {
  SerialChain chain;
  chain.name = object.value("name", std::string("robot"));
  if (object.contains("base")) {
    chain.base = readPose(object.at("base"));
  }
  if (object.contains("tool")) {
    chain.tool = readPose(object.at("tool"));
  }
  if (!object.contains("joints")) {
    throw std::runtime_error("robot must contain a joints array");
  }
  for (const json& joint_json : object.at("joints")) {
    chain.joints.push_back(readJoint(joint_json));
  }

  if (object.contains("link_geometries")) {
    for (const json& geometry_json : object.at("link_geometries")) {
      chain.link_geometries.push_back(readGeometry(geometry_json));
    }
    if (chain.link_geometries.size() != chain.joints.size()) {
      throw std::runtime_error("link_geometries size must equal joint count");
    }
  } else if (object.contains("link_radius")) {
    const double radius = object.value("link_radius", 0.05);
    chain.link_geometries.resize(chain.joints.size());
    for (std::size_t i = 0; i < chain.joints.size(); ++i) {
      Geometry capsule;
      capsule.type = Geometry::Type::Capsule;
      capsule.radius = radius;
      capsule.a = Vec3::Zero();
      if (i + 1 < chain.joints.size()) {
        capsule.b = chain.joints[i + 1].origin.translation();
      } else {
        capsule.b = chain.tool.translation();
      }
      if (capsule.b.norm() < 1e-9) {
        capsule.b = Vec3(0.0, 0.0, 0.05);
      }
      chain.link_geometries[i] = capsule;
    }
  }

  if (object.contains("tool_geometry")) {
    chain.tool_geometry = readGeometry(object.at("tool_geometry"));
  }
  return chain;
}

void parseScene(const json& root, SceneConfig* config) {
  if (!root.contains("robot")) {
    throw std::runtime_error("config must contain a robot object");
  }
  config->scene.robot = readChain(root.at("robot"));

  if (root.contains("part")) {
    const json& part_json = root.at("part");
    config->scene.part.name = part_json.value("name", std::string("part"));
    if (part_json.contains("geometry")) {
      config->scene.part.geometry = readGeometry(part_json.at("geometry"));
    }
    if (part_json.contains("pose")) {
      config->scene.part.pose = readPose(part_json.at("pose"));
    }
  }

  if (root.contains("obstacles")) {
    for (const json& obstacle_json : root.at("obstacles")) {
      Obstacle obstacle;
      obstacle.name = obstacle_json.value("name", std::string("obstacle"));
      if (obstacle_json.contains("geometry")) {
        obstacle.geometry = readGeometry(obstacle_json.at("geometry"));
      }
      if (obstacle_json.contains("pose")) {
        obstacle.pose = readPose(obstacle_json.at("pose"));
      }
      config->scene.obstacles.push_back(std::move(obstacle));
    }
  }

  config->scene.has_ground = root.value("has_ground", true);
  config->scene.ground_z = root.value("ground_z", 0.0);
}

void parseSurface(const json& object, TargetSurface* surface) {
  surface->name = object.value("name", std::string("surface"));
  surface->origin = readVec3(object, "origin", Vec3::Zero());
  surface->u_axis = readVec3(object, "u_axis", Vec3::UnitX());
  surface->v_axis = readVec3(object, "v_axis", Vec3::UnitY());
  if (object.contains("size")) {
    const auto size = object.at("size").get<std::vector<double>>();
    if (size.size() != 2) {
      throw std::runtime_error("surface size must be [size_u, size_v]");
    }
    surface->size_u = size[0];
    surface->size_v = size[1];
  } else {
    surface->size_u = object.value("size_u", surface->size_u);
    surface->size_v = object.value("size_v", surface->size_v);
  }
  surface->normal = readVec3(object, "normal", Vec3::Zero());
}

void parseScanner(const json& object, ScannerParams* scanner) {
  scanner->standoff = object.value("standoff", scanner->standoff);
  scanner->min_standoff = object.value("min_standoff", scanner->min_standoff);
  scanner->max_standoff = object.value("max_standoff", scanner->max_standoff);
  scanner->fov_width = object.value("fov_width", scanner->fov_width);
  scanner->overlap = object.value("overlap", scanner->overlap);
  scanner->sample_spacing = object.value("sample_spacing", scanner->sample_spacing);
  scanner->scan_speed = object.value("scan_speed", scanner->scan_speed);
  scanner->max_incidence_deg = object.value("max_incidence_deg", scanner->max_incidence_deg);
}

void parsePoseTarget(const json& object, double default_dwell, PoseTarget* target) {
  if (!object.is_object()) {
    throw std::runtime_error("each pose entry must be an object");
  }
  target->name = object.value("name", std::string("pose"));
  if (object.contains("pose") && object.at("pose").is_object()) {
    target->pose = readPose(object.at("pose"));
  } else {
    target->pose = readPose(object);
  }
  target->dwell_time = object.value("dwell_time", object.value("dwell", default_dwell));
  if (target->dwell_time < 0.0) {
    target->dwell_time = 0.0;
  }
}

void parsePoseGroups(const json& groups, double default_dwell, SceneConfig* config) {
  if (!groups.is_array()) {
    throw std::runtime_error("pose groups must be an array");
  }
  for (const json& group_json : groups) {
    if (!group_json.is_object()) {
      throw std::runtime_error("each pose group must be an object");
    }
    PoseGroup group;
    group.name = group_json.value("name", std::string("group"));
    group.optimize_order = group_json.value("optimize_order", true);
    if (!group_json.contains("poses")) {
      throw std::runtime_error("pose group '" + group.name + "' must contain a poses array");
    }
    const json& poses = group_json.at("poses");
    if (!poses.is_array()) {
      throw std::runtime_error("poses in group '" + group.name + "' must be an array");
    }
    const double group_dwell =
        group_json.value("dwell_time", group_json.value("default_dwell", default_dwell));
    for (const json& pose_json : poses) {
      PoseTarget target;
      parsePoseTarget(pose_json, group_dwell, &target);
      group.poses.push_back(std::move(target));
    }
    config->pose_groups.push_back(std::move(group));
  }
}

void parseTask(const json& root, SceneConfig* config) {
  std::string type = "surface_coverage";
  double default_dwell = 0.5;
  const json* pose_array = nullptr;
  const json* groups_array = nullptr;

  if (root.contains("task")) {
    const json& task = root.at("task");
    if (task.is_string()) {
      type = task.get<std::string>();
    } else if (task.is_object()) {
      type = task.value("type", std::string("surface_coverage"));
      default_dwell = task.value("dwell_time", task.value("default_dwell", default_dwell));
      if (task.contains("poses")) {
        pose_array = &task.at("poses");
      }
      if (task.contains("groups")) {
        groups_array = &task.at("groups");
      }
      config->options.return_home =
          task.value("return_home", config->options.return_home);
      config->options.require_all_poses =
          task.value("require_all_poses", config->options.require_all_poses);
    } else {
      throw std::runtime_error("task must be a string or an object");
    }
  } else if (root.contains("groups")) {
    type = "pose_groups";
    groups_array = &root.at("groups");
  } else if (root.contains("poses")) {
    type = "pose_sequence";
    pose_array = &root.at("poses");
  }

  if (type == "pose_groups" || type == "groups" || type == "multi_pose_sequence" ||
      groups_array != nullptr) {
    config->task = TaskType::PoseGroups;
    config->default_pose_dwell = default_dwell;
    if (groups_array == nullptr && root.contains("groups")) {
      groups_array = &root.at("groups");
    }
    if (groups_array == nullptr) {
      throw std::runtime_error("pose groups task must contain a groups array");
    }
    parsePoseGroups(*groups_array, default_dwell, config);
    if (config->pose_groups.empty()) {
      throw std::runtime_error("pose groups must contain at least one group");
    }
    return;
  }

  if (type == "pose_sequence" || type == "pose" || type == "poses" ||
      type == "pose_list") {
    config->task = TaskType::PoseSequence;
    config->default_pose_dwell = default_dwell;
    if (pose_array == nullptr && root.contains("poses")) {
      pose_array = &root.at("poses");
    }
    if (pose_array == nullptr) {
      throw std::runtime_error("pose sequence task must contain a poses array");
    }
    if (!pose_array->is_array() || pose_array->empty()) {
      throw std::runtime_error("poses must be a non-empty array");
    }
    for (const json& pose_json : *pose_array) {
      PoseTarget target;
      parsePoseTarget(pose_json, default_dwell, &target);
      config->poses.push_back(std::move(target));
    }
    return;
  }

  if (type == "surface" || type == "surface_coverage" || type == "coverage") {
    config->task = TaskType::SurfaceCoverage;
    return;
  }
  throw std::runtime_error("unknown task type: " + type);
}

void parseOptions(const json& object, PlanningOptions* options) {
  options->collision_margin = object.value("collision_margin", options->collision_margin);
  options->collision_step = object.value("collision_step", options->collision_step);
  options->coverage_cell = object.value("coverage_cell", options->coverage_cell);
  options->ik_max_iterations = object.value("ik_max_iterations", options->ik_max_iterations);
  options->ik_seed_attempts = object.value("ik_seed_attempts", options->ik_seed_attempts);
  options->max_scan_joint_step = object.value("max_scan_joint_step", options->max_scan_joint_step);
  options->optimize_order = object.value("optimize_order", options->optimize_order);
  options->require_full_coverage =
      object.value("require_full_coverage", options->require_full_coverage);
  options->sweep_along_u = object.value("sweep_along_u", options->sweep_along_u);
  options->verbose = object.value("verbose", options->verbose);
  if (object.contains("motion_backend")) {
    const std::string name = object.at("motion_backend").get<std::string>();
    if (!parseMotionBackend(name, &options->motion_backend)) {
      throw std::runtime_error("unknown motion_backend: " + name);
    }
  }
  options->ompl_time_limit = object.value("ompl_time_limit", options->ompl_time_limit);
  options->ompl_range = object.value("ompl_range", options->ompl_range);
  options->ompl_seed = object.value("ompl_seed", options->ompl_seed);
  options->ompl_simplify = object.value("ompl_simplify", options->ompl_simplify);
  options->return_home = object.value("return_home", options->return_home);
  options->require_all_poses = object.value("require_all_poses", options->require_all_poses);
  if (object.contains("ik_backend")) {
    const std::string backend = toLower(object.at("ik_backend").get<std::string>());
    if (backend == "internal" || backend == "analytic" || backend == "dls") {
      options->ik_backend = IKBackend::Internal;
    } else if (backend == "kdl" || backend == "orocos_kdl") {
      options->ik_backend = IKBackend::Kdl;
    } else {
      throw std::runtime_error("unknown ik_backend: " + backend);
    }
  }
  if (object.contains("collision_backend")) {
    const std::string backend = toLower(object.at("collision_backend").get<std::string>());
    if (backend == "internal" || backend == "primitive") {
      options->collision_backend = CollisionBackend::Internal;
    } else if (backend == "fcl") {
      options->collision_backend = CollisionBackend::Fcl;
    } else {
      throw std::runtime_error("unknown collision_backend: " + backend);
    }
  }
}

json vec3Json(const Vec3& value) { return json::array({value.x(), value.y(), value.z()}); }

json vecJson(const VecX& value) {
  json array = json::array();
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    array.push_back(value[i]);
  }
  return array;
}

json waypointJson(const PlannedWaypoint& waypoint) {
  return json{{"time", waypoint.time}, {"q", vecJson(waypoint.q)}, {"tcp", vec3Json(waypoint.tcp)}};
}

}  // namespace

SceneConfig parseSceneConfig(const std::string& json_text) {
  SceneConfig config;
  json root;
  try {
    root = json::parse(json_text);
  } catch (const std::exception& exception) {
    throw std::runtime_error(std::string("failed to parse scene JSON: ") + exception.what());
  }
  if (!root.is_object()) {
    throw std::runtime_error("scene JSON root must be an object");
  }

  parseScene(root, &config);
  parseTask(root, &config);
  config.options.return_home = root.value("return_home", config.options.return_home);
  config.options.require_all_poses =
      root.value("require_all_poses", config.options.require_all_poses);
  if (config.task == TaskType::SurfaceCoverage) {
    if (!root.contains("surface")) {
      throw std::runtime_error("config must contain a surface object");
    }
    parseSurface(root.at("surface"), &config.surface);
  }
  if (root.contains("scanner")) {
    parseScanner(root.at("scanner"), &config.scanner);
  }
  if (root.contains("options")) {
    parseOptions(root.at("options"), &config.options);
  }
  return config;
}

PlanResult planScene(const SceneConfig& config) {
  if (config.task == TaskType::PoseGroups) {
    return Planner::planPoseGroups(config.scene, config.pose_groups, config.options);
  }
  if (config.task == TaskType::PoseSequence) {
    return Planner::planPoseSequence(config.scene, config.poses, config.options);
  }
  return Planner::plan(config.scene, config.surface, config.scanner, config.options);
}

SceneConfig loadSceneConfig(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open scene config: " + path);
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return parseSceneConfig(buffer.str());
}

std::string planResultToJson(const SceneConfig& config, const PlanResult& result) {
  json root;
  root["success"] = result.success;
  root["task"] = result.task_type;
  root["message"] = result.message;
  root["planner"] = motionBackendName(config.options.motion_backend);
  root["ik_backend"] = ikBackendName(config.options.ik_backend);
  root["collision_backend"] = collisionBackendName(config.options.collision_backend);
  root["ompl_available"] = isOmplAvailable();
  root["kdl_available"] = isKdlAvailable();
  root["fcl_available"] = isFclAvailable();
  root["tesseract_available"] = isTesseractAvailable();

  const PlanMetrics& metrics = result.metrics;
  root["metrics"] = {
      {"total_time_s", metrics.total_time_s},
      {"scan_time_s", metrics.scan_time_s},
      {"transfer_time_s", metrics.transfer_time_s},
      {"dwell_time_s", metrics.dwell_time_s},
      {"tcp_path_m", metrics.tcp_path_m},
      {"agv_distance_m", metrics.agv_distance_m},
      {"external_revolute_rad", metrics.external_revolute_rad},
      {"external_prismatic_m", metrics.external_prismatic_m},
      {"target_area_m2", metrics.target_area_m2},
      {"covered_area_m2", metrics.covered_area_m2},
      {"coverage_ratio", metrics.coverage_ratio},
      {"strips_generated", metrics.strips_generated},
      {"runs_planned", metrics.runs_planned},
      {"samples_total", metrics.samples_total},
      {"samples_feasible", metrics.samples_feasible},
      {"poses_total", metrics.poses_total},
      {"poses_reachable", metrics.poses_reachable},
      {"pose_groups_total", metrics.pose_groups_total},
      {"pose_groups_planned", metrics.pose_groups_planned},
      {"collision_free", metrics.collision_free},
      {"order_valid", metrics.order_valid},
      {"full_coverage", metrics.full_coverage},
  };

  json scene_summary = {
      {"task", result.task_type},
      {"robot", config.scene.robot.name},
      {"joints", config.scene.robot.joints.size()},
  };
  if (config.task == TaskType::SurfaceCoverage) {
    scene_summary["surface"] = config.surface.name;
    scene_summary["surface_size"] =
        json::array({config.surface.size_u, config.surface.size_v});
    scene_summary["scanner_standoff"] = config.scanner.standoff;
    scene_summary["scanner_fov_width"] = config.scanner.fov_width;
  } else {
    if (config.task == TaskType::PoseGroups) {
      std::size_t pose_count = 0;
      for (const PoseGroup& group : config.pose_groups) {
        pose_count += group.poses.size();
      }
      scene_summary["group_count"] = config.pose_groups.size();
      scene_summary["pose_count"] = pose_count;
    } else {
      scene_summary["pose_count"] = config.poses.size();
    }
    scene_summary["return_home"] = config.options.return_home;
  }
  root["scene_summary"] = std::move(scene_summary);

  json runs = json::array();
  for (const PlannedRun& run : result.runs) {
    json run_json;
    run_json["id"] = run.id;
    run_json["strip_id"] = run.strip_id;
    run_json["reversed"] = run.reversed;
    run_json["start_time"] = run.start_time;
    run_json["end_time"] = run.end_time;
    run_json["duration"] = run.duration;
    run_json["length_m"] = run.length_m;
    json waypoints = json::array();
    for (const PlannedWaypoint& waypoint : run.waypoints) {
      waypoints.push_back(waypointJson(waypoint));
    }
    run_json["waypoints"] = std::move(waypoints);
    runs.push_back(std::move(run_json));
  }
  root["runs"] = std::move(runs);

  json pose_visits = json::array();
  for (const PlannedPoseVisit& visit : result.pose_visits) {
    pose_visits.push_back({
        {"name", visit.name},
        {"input_index", visit.input_index},
        {"order", visit.order},
        {"group_index", visit.group_index},
        {"group_pose_index", visit.group_pose_index},
        {"group_name", visit.group_name},
        {"q", vecJson(visit.q)},
        {"tcp", vec3Json(visit.tcp)},
        {"arrival_time", visit.arrival_time},
        {"dwell_time", visit.dwell_time},
    });
  }
  root["pose_visits"] = std::move(pose_visits);

  json pose_groups = json::array();
  for (const PlannedPoseGroup& group : result.pose_groups) {
    pose_groups.push_back({
        {"name", group.name},
        {"input_index", group.input_index},
        {"poses_total", group.poses_total},
        {"poses_visited", group.poses_visited},
        {"optimize_order", group.optimize_order},
        {"start_time", group.start_time},
        {"end_time", group.end_time},
    });
  }
  root["pose_groups"] = std::move(pose_groups);

  json transitions = json::array();
  for (const PlannedTransition& transition : result.transitions) {
    json transition_json;
    transition_json["from_run"] = transition.from_run;
    transition_json["to_run"] = transition.to_run;
    transition_json["from_pose"] = transition.from_pose;
    transition_json["to_pose"] = transition.to_pose;
    transition_json["start_time"] = transition.start_time;
    transition_json["end_time"] = transition.end_time;
    transition_json["duration"] = transition.duration;
    transition_json["length_m"] = transition.length_m;
    json waypoints = json::array();
    for (const PlannedWaypoint& waypoint : transition.waypoints) {
      waypoints.push_back(waypointJson(waypoint));
    }
    transition_json["waypoints"] = std::move(waypoints);
    transitions.push_back(std::move(transition_json));
  }
  root["transitions"] = std::move(transitions);
  root["warnings"] = result.warnings;

  return root.dump(2);
}

bool savePlanJson(const std::string& path, const SceneConfig& config, const PlanResult& result) {
  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    return false;
  }
  stream << planResultToJson(config, result);
  return static_cast<bool>(stream);
}

}  // namespace rp
