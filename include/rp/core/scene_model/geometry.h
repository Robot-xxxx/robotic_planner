#pragma once

#include "rp/core/types.h"

#include <array>
#include <vector>

namespace rp {

struct Geometry {
  enum class Type { None, Sphere, Capsule, Box, Mesh };

  Type type = Type::None;
  Isometry local = Isometry::Identity();

  Vec3 center = Vec3::Zero();
  Vec3 a = Vec3::Zero();
  Vec3 b = Vec3::Zero();
  double radius = 0.0;
  Vec3 half_extents = Vec3::Ones() * 0.5;

  std::vector<Vec3> vertices;
  std::vector<std::array<int, 3>> triangles;

  bool empty() const { return type == Type::None; }
};

}  // namespace rp
