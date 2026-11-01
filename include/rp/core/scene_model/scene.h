#pragma once

#include "rp/core/scene_model/geometry.h"
#include "rp/core/types.h"

#include <string>
#include <vector>

namespace rp {

enum class JointType { Revolute, Prismatic };

struct Joint {
  std::string name;
  JointType type = JointType::Revolute;
  Vec3 axis = Vec3::UnitZ();
  Isometry origin = Isometry::Identity();
  double lower = -kPi;
  double upper = kPi;
  double home = 0.0;
  double velocity_limit = 1.5;
  double acceleration_limit = 3.0;
  bool external = false;
};

struct SerialChain {
  std::string name;
  Isometry base = Isometry::Identity();
  Isometry tool = Isometry::Identity();
  std::vector<Joint> joints;
  std::vector<Geometry> link_geometries;
  Geometry tool_geometry;
};

struct Part {
  std::string name = "part";
  Geometry geometry;
  Isometry pose = Isometry::Identity();
};

struct Obstacle {
  std::string name = "obstacle";
  Geometry geometry;
  Isometry pose = Isometry::Identity();
};

struct SceneModel {
  SerialChain robot;
  Part part;
  std::vector<Obstacle> obstacles;
  bool has_ground = true;
  double ground_z = 0.0;
};

bool validateChain(const SerialChain& chain, std::string* error);
VecX homeConfiguration(const SerialChain& chain);

}  // namespace rp
