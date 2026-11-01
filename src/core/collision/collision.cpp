#include "rp/core/collision/collision.h"

#include "rp/core/kinematics/kinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#ifdef RP_WITH_FCL
#include <fcl/fcl.h>
#endif

namespace rp {

namespace {

Vec3 worldPoint(const Isometry& pose, const Vec3& local) { return pose * local; }

double pointSegmentDistanceSquared(const Vec3& p, const Vec3& a, const Vec3& b) {
  const Vec3 ab = b - a;
  const double denominator = ab.squaredNorm();
  if (denominator < kEpsilon) {
    return (p - a).squaredNorm();
  }
  double t = (p - a).dot(ab) / denominator;
  t = std::min(1.0, std::max(0.0, t));
  return (p - (a + t * ab)).squaredNorm();
}

double segmentSegmentDistanceSquared(const Vec3& p1, const Vec3& q1, const Vec3& p2,
                                     const Vec3& q2) {
  const Vec3 d1 = q1 - p1;
  const Vec3 d2 = q2 - p2;
  const Vec3 r = p1 - p2;
  const double a = d1.dot(d1);
  const double e = d2.dot(d2);
  const double f = d2.dot(r);

  double s = 0.0;
  double t = 0.0;

  if (a <= kEpsilon && e <= kEpsilon) {
    return r.squaredNorm();
  }
  if (a <= kEpsilon) {
    t = std::min(1.0, std::max(0.0, f / e));
  } else {
    const double c = d1.dot(r);
    if (e <= kEpsilon) {
      s = std::min(1.0, std::max(0.0, -c / a));
    } else {
      const double b = d1.dot(d2);
      const double denominator = a * e - b * b;
      if (denominator > kEpsilon) {
        s = std::min(1.0, std::max(0.0, (b * f - c * e) / denominator));
      }
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::min(1.0, std::max(0.0, -c / a));
      } else if (t > 1.0) {
        t = 1.0;
        s = std::min(1.0, std::max(0.0, (b - c) / a));
      }
    }
  }

  const Vec3 c1 = p1 + d1 * s;
  const Vec3 c2 = p2 + d2 * t;
  return (c1 - c2).squaredNorm();
}

double pointTriangleDistanceSquared(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
  const Vec3 ab = b - a;
  const Vec3 ac = c - a;
  const Vec3 ap = p - a;
  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return ap.squaredNorm();
  }

  const Vec3 bp = p - b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return bp.squaredNorm();
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return (p - (a + v * ab)).squaredNorm();
  }

  const Vec3 cp = p - c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return cp.squaredNorm();
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    return (p - (a + w * ac)).squaredNorm();
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return (p - (b + w * (c - b))).squaredNorm();
  }

  const double denominator = 1.0 / (va + vb + vc);
  const double v = vb * denominator;
  const double w = vc * denominator;
  const Vec3 closest = a + ab * v + ac * w;
  return (p - closest).squaredNorm();
}

bool segmentIntersectsTriangle(const Vec3& p0, const Vec3& p1, const Vec3& a, const Vec3& b,
                               const Vec3& c) {
  const Vec3 direction = p1 - p0;
  const Vec3 edge1 = b - a;
  const Vec3 edge2 = c - a;
  const Vec3 h = direction.cross(edge2);
  const double det = edge1.dot(h);
  if (std::abs(det) < 1e-12) {
    return false;
  }
  const double inv_det = 1.0 / det;
  const Vec3 s = p0 - a;
  const double u = inv_det * s.dot(h);
  if (u < 0.0 || u > 1.0) {
    return false;
  }
  const Vec3 q = s.cross(edge1);
  const double v = inv_det * direction.dot(q);
  if (v < 0.0 || u + v > 1.0) {
    return false;
  }
  const double t = inv_det * edge2.dot(q);
  return t >= 0.0 && t <= 1.0;
}

double segmentTriangleDistanceSquared(const Vec3& p0, const Vec3& p1, const Vec3& a, const Vec3& b,
                                      const Vec3& c) {
  if (segmentIntersectsTriangle(p0, p1, a, b, c)) {
    return 0.0;
  }
  double best = std::min(pointTriangleDistanceSquared(p0, a, b, c),
                         pointTriangleDistanceSquared(p1, a, b, c));
  best = std::min(best, segmentSegmentDistanceSquared(p0, p1, a, b));
  best = std::min(best, segmentSegmentDistanceSquared(p0, p1, b, c));
  best = std::min(best, segmentSegmentDistanceSquared(p0, p1, c, a));
  return best;
}

double pointAabbDistanceSquared(const Vec3& p, const Vec3& half_extents) {
  const Vec3 closest = p.cwiseMax(-half_extents).cwiseMin(half_extents);
  return (p - closest).squaredNorm();
}

bool segmentIntersectsAabb(const Vec3& p0, const Vec3& p1, const Vec3& half_extents) {
  const Vec3 d = p1 - p0;
  double t_min = 0.0;
  double t_max = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(d[axis]) < kEpsilon) {
      if (p0[axis] < -half_extents[axis] || p0[axis] > half_extents[axis]) {
        return false;
      }
      continue;
    }
    double t1 = (-half_extents[axis] - p0[axis]) / d[axis];
    double t2 = (half_extents[axis] - p0[axis]) / d[axis];
    if (t1 > t2) {
      std::swap(t1, t2);
    }
    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
    if (t_min > t_max) {
      return false;
    }
  }
  return true;
}

double segmentAabbDistanceSquared(const Vec3& p0, const Vec3& p1, const Vec3& half_extents) {
  if (segmentIntersectsAabb(p0, p1, half_extents)) {
    return 0.0;
  }
  double lo = 0.0;
  double hi = 1.0;
  for (int iteration = 0; iteration < 64; ++iteration) {
    const double m1 = lo + (hi - lo) / 3.0;
    const double m2 = hi - (hi - lo) / 3.0;
    const Vec3 q1 = p0 + (p1 - p0) * m1;
    const Vec3 q2 = p0 + (p1 - p0) * m2;
    if (pointAabbDistanceSquared(q1, half_extents) <
        pointAabbDistanceSquared(q2, half_extents)) {
      hi = m2;
    } else {
      lo = m1;
    }
  }
  const Vec3 q = p0 + (p1 - p0) * (0.5 * (lo + hi));
  return pointAabbDistanceSquared(q, half_extents);
}

double capsuleCapsuleDistance(const Geometry& a, const Isometry& pose_a, const Geometry& b,
                              const Isometry& pose_b) {
  const Vec3 a0 = worldPoint(pose_a, a.a);
  const Vec3 a1 = worldPoint(pose_a, a.b);
  const Vec3 b0 = worldPoint(pose_b, b.a);
  const Vec3 b1 = worldPoint(pose_b, b.b);
  const double distance = std::sqrt(segmentSegmentDistanceSquared(a0, a1, b0, b1));
  return distance - a.radius - b.radius;
}

double capsuleSphereDistance(const Geometry& capsule, const Isometry& pose_capsule,
                             const Geometry& sphere, const Isometry& pose_sphere) {
  const Vec3 a = worldPoint(pose_capsule, capsule.a);
  const Vec3 b = worldPoint(pose_capsule, capsule.b);
  const Vec3 center = worldPoint(pose_sphere, sphere.center);
  return std::sqrt(pointSegmentDistanceSquared(center, a, b)) - capsule.radius - sphere.radius;
}

double capsuleBoxDistance(const Geometry& capsule, const Isometry& pose_capsule,
                          const Geometry& box, const Isometry& pose_box) {
  const Isometry inverse = pose_box.inverse();
  const Vec3 a = inverse * worldPoint(pose_capsule, capsule.a) - box.center;
  const Vec3 b = inverse * worldPoint(pose_capsule, capsule.b) - box.center;
  return std::sqrt(segmentAabbDistanceSquared(a, b, box.half_extents)) - capsule.radius;
}

double sphereBoxDistance(const Geometry& sphere, const Isometry& pose_sphere, const Geometry& box,
                         const Isometry& pose_box) {
  const Vec3 center = pose_box.inverse() * worldPoint(pose_sphere, sphere.center) - box.center;
  return std::sqrt(pointAabbDistanceSquared(center, box.half_extents)) - sphere.radius;
}

double capsuleMeshDistance(const Geometry& capsule, const Isometry& pose_capsule,
                           const Geometry& mesh, const Isometry& pose_mesh) {
  const Vec3 a = worldPoint(pose_capsule, capsule.a);
  const Vec3 b = worldPoint(pose_capsule, capsule.b);
  if (mesh.triangles.empty()) {
    double best = std::numeric_limits<double>::max();
    for (const Vec3& vertex : mesh.vertices) {
      best = std::min(best, pointSegmentDistanceSquared(worldPoint(pose_mesh, vertex), a, b));
    }
    return std::sqrt(best) - capsule.radius;
  }
  double best = std::numeric_limits<double>::max();
  for (const auto& triangle : mesh.triangles) {
    if (triangle[0] < 0 || triangle[1] < 0 || triangle[2] < 0 ||
        triangle[0] >= static_cast<int>(mesh.vertices.size()) ||
        triangle[1] >= static_cast<int>(mesh.vertices.size()) ||
        triangle[2] >= static_cast<int>(mesh.vertices.size())) {
      continue;
    }
    const Vec3 v0 = worldPoint(pose_mesh, mesh.vertices[static_cast<std::size_t>(triangle[0])]);
    const Vec3 v1 = worldPoint(pose_mesh, mesh.vertices[static_cast<std::size_t>(triangle[1])]);
    const Vec3 v2 = worldPoint(pose_mesh, mesh.vertices[static_cast<std::size_t>(triangle[2])]);
    best = std::min(best, segmentTriangleDistanceSquared(a, b, v0, v1, v2));
  }
  return std::sqrt(best) - capsule.radius;
}

double sphereMeshDistance(const Geometry& sphere, const Isometry& pose_sphere, const Geometry& mesh,
                          const Isometry& pose_mesh) {
  const Vec3 center = worldPoint(pose_sphere, sphere.center);
  if (mesh.triangles.empty()) {
    double best = std::numeric_limits<double>::max();
    for (const Vec3& vertex : mesh.vertices) {
      best = std::min(best, (center - worldPoint(pose_mesh, vertex)).squaredNorm());
    }
    return std::sqrt(best) - sphere.radius;
  }
  double best = std::numeric_limits<double>::max();
  for (const auto& triangle : mesh.triangles) {
    if (triangle[0] < 0 || triangle[1] < 0 || triangle[2] < 0 ||
        triangle[0] >= static_cast<int>(mesh.vertices.size()) ||
        triangle[1] >= static_cast<int>(mesh.vertices.size()) ||
        triangle[2] >= static_cast<int>(mesh.vertices.size())) {
      continue;
    }
    const Vec3 v0 = worldPoint(pose_mesh, mesh.vertices[static_cast<std::size_t>(triangle[0])]);
    const Vec3 v1 = worldPoint(pose_mesh, mesh.vertices[static_cast<std::size_t>(triangle[1])]);
    const Vec3 v2 = worldPoint(pose_mesh, mesh.vertices[static_cast<std::size_t>(triangle[2])]);
    best = std::min(best, pointTriangleDistanceSquared(center, v0, v1, v2));
  }
  return std::sqrt(best) - sphere.radius;
}

bool isCapsule(const Geometry& g) { return g.type == Geometry::Type::Capsule; }
bool isSphere(const Geometry& g) { return g.type == Geometry::Type::Sphere; }
bool isBox(const Geometry& g) { return g.type == Geometry::Type::Box; }
bool isMesh(const Geometry& g) { return g.type == Geometry::Type::Mesh; }

#ifdef RP_WITH_FCL
struct FclBody {
  std::string name;
  std::shared_ptr<fcl::CollisionGeometryd> geometry;
  fcl::Transform3d transform = fcl::Transform3d::Identity();
  bool is_robot = false;
  int link_index = -1;
};

fcl::Transform3d toFclTransform(const Isometry& transform) {
  fcl::Transform3d result = fcl::Transform3d::Identity();
  result.linear() = transform.linear();
  result.translation() = transform.translation();
  return result;
}

std::shared_ptr<fcl::CollisionGeometryd> makeFclGeometry(const Geometry& geometry,
                                                         fcl::Transform3d* transform) {
  if (geometry.empty()) {
    return nullptr;
  }
  switch (geometry.type) {
    case Geometry::Type::Sphere: {
      auto shape = std::make_shared<fcl::Sphered>(geometry.radius);
      transform->translation() += transform->linear() * geometry.center;
      return shape;
    }
    case Geometry::Type::Box: {
      auto shape = std::make_shared<fcl::Boxd>(2.0 * geometry.half_extents.x(),
                                               2.0 * geometry.half_extents.y(),
                                               2.0 * geometry.half_extents.z());
      transform->translation() += transform->linear() * geometry.center;
      return shape;
    }
    case Geometry::Type::Capsule: {
      const Vec3 delta = geometry.b - geometry.a;
      const double length = delta.norm();
      if (length < 1e-9) {
        auto shape = std::make_shared<fcl::Sphered>(geometry.radius);
        transform->translation() += geometry.a;
        return shape;
      }
      auto shape = std::make_shared<fcl::Capsuled>(geometry.radius, length);
      const Vec3 mid = (geometry.a + geometry.b) * 0.5;
      const Eigen::Quaterniond rotation =
          Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(), delta.normalized());
      const Isometry local = makeIsometry(mid, rotation.toRotationMatrix());
      *transform = (*transform) * toFclTransform(local);
      return shape;
    }
    case Geometry::Type::Mesh: {
      auto mesh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
      mesh->beginModel();
      for (const auto& triangle : geometry.triangles) {
        if (triangle[0] < 0 || triangle[1] < 0 || triangle[2] < 0 ||
            triangle[0] >= static_cast<int>(geometry.vertices.size()) ||
            triangle[1] >= static_cast<int>(geometry.vertices.size()) ||
            triangle[2] >= static_cast<int>(geometry.vertices.size())) {
          continue;
        }
        // FCL can build a BVH directly from triangle points.
        mesh->addTriangle(
            fcl::Vector3d(geometry.vertices[static_cast<std::size_t>(triangle[0])].x(),
                          geometry.vertices[static_cast<std::size_t>(triangle[0])].y(),
                          geometry.vertices[static_cast<std::size_t>(triangle[0])].z()),
            fcl::Vector3d(geometry.vertices[static_cast<std::size_t>(triangle[1])].x(),
                          geometry.vertices[static_cast<std::size_t>(triangle[1])].y(),
                          geometry.vertices[static_cast<std::size_t>(triangle[1])].z()),
            fcl::Vector3d(geometry.vertices[static_cast<std::size_t>(triangle[2])].x(),
                          geometry.vertices[static_cast<std::size_t>(triangle[2])].y(),
                          geometry.vertices[static_cast<std::size_t>(triangle[2])].z()));
      }
      mesh->endModel();
      if (mesh->num_tris == 0) {
        return nullptr;
      }
      return mesh;
    }
    case Geometry::Type::None:
    default:
      return nullptr;
  }
}
#endif

}  // namespace

double geometryDistance(const Geometry& a, const Isometry& pose_a, const Geometry& b,
                        const Isometry& pose_b) {
  if (a.empty() || b.empty()) {
    return 1e9;
  }
  if (isCapsule(a) && isCapsule(b)) {
    return capsuleCapsuleDistance(a, pose_a, b, pose_b);
  }
  if (isCapsule(a) && isSphere(b)) {
    return capsuleSphereDistance(a, pose_a, b, pose_b);
  }
  if (isSphere(a) && isCapsule(b)) {
    return capsuleSphereDistance(b, pose_b, a, pose_a);
  }
  if (isSphere(a) && isSphere(b)) {
    const Vec3 ca = worldPoint(pose_a, a.center);
    const Vec3 cb = worldPoint(pose_b, b.center);
    return (ca - cb).norm() - a.radius - b.radius;
  }
  if (isCapsule(a) && isBox(b)) {
    return capsuleBoxDistance(a, pose_a, b, pose_b);
  }
  if (isBox(a) && isCapsule(b)) {
    return capsuleBoxDistance(b, pose_b, a, pose_a);
  }
  if (isSphere(a) && isBox(b)) {
    return sphereBoxDistance(a, pose_a, b, pose_b);
  }
  if (isBox(a) && isSphere(b)) {
    return sphereBoxDistance(b, pose_b, a, pose_a);
  }
  if (isCapsule(a) && isMesh(b)) {
    return capsuleMeshDistance(a, pose_a, b, pose_b);
  }
  if (isMesh(a) && isCapsule(b)) {
    return capsuleMeshDistance(b, pose_b, a, pose_a);
  }
  if (isSphere(a) && isMesh(b)) {
    return sphereMeshDistance(a, pose_a, b, pose_b);
  }
  if (isMesh(a) && isSphere(b)) {
    return sphereMeshDistance(b, pose_b, a, pose_a);
  }
  return 1e9;
}

double geometryGroundDistance(const Geometry& geometry, const Isometry& pose, double ground_z) {
  if (geometry.empty()) {
    return 1e9;
  }
  if (isCapsule(geometry)) {
    const double z0 = worldPoint(pose, geometry.a).z();
    const double z1 = worldPoint(pose, geometry.b).z();
    return std::min(z0, z1) - geometry.radius - ground_z;
  }
  if (isSphere(geometry)) {
    return worldPoint(pose, geometry.center).z() - geometry.radius - ground_z;
  }
  if (isBox(geometry)) {
    const Vec3 center = worldPoint(pose, geometry.center);
    double extent_z = 0.0;
    for (int i = 0; i < 3; ++i) {
      extent_z += std::abs(pose.linear()(2, i)) * geometry.half_extents[i];
    }
    return center.z() - extent_z - ground_z;
  }
  if (isMesh(geometry)) {
    double min_z = std::numeric_limits<double>::max();
    for (const Vec3& vertex : geometry.vertices) {
      min_z = std::min(min_z, worldPoint(pose, vertex).z());
    }
    if (min_z == std::numeric_limits<double>::max()) {
      return 1e9;
    }
    return min_z - ground_z;
  }
  return 1e9;
}

CollisionChecker::CollisionChecker(const SceneModel& scene) : scene_(scene) {
  options_.check_ground = scene.has_ground;
  options_.ground_z = scene.ground_z;
}

CollisionChecker::CollisionChecker(const SceneModel& scene, const CollisionOptions& options)
    : scene_(scene), options_(options) {}

std::vector<CollisionChecker::WorldBody> CollisionChecker::buildBodies(const VecX& q) const {
  std::vector<WorldBody> bodies;
  const FKResult fk = Kinematics::forward(scene_.robot, q);

  for (std::size_t i = 0; i < fk.link_frames.size(); ++i) {
    if (i >= scene_.robot.link_geometries.size()) {
      continue;
    }
    const Geometry& geometry = scene_.robot.link_geometries[i];
    if (geometry.empty()) {
      continue;
    }
    WorldBody body;
    body.name = scene_.robot.name + "/" + scene_.robot.joints[i].name;
    body.geometry = geometry;
    body.pose = fk.link_frames[i] * geometry.local;
    body.is_robot = true;
    body.link_index = static_cast<int>(i);
    bodies.push_back(body);
  }

  if (!scene_.robot.tool_geometry.empty()) {
    WorldBody body;
    body.name = scene_.robot.name + "/tool";
    body.geometry = scene_.robot.tool_geometry;
    body.pose = fk.tool * scene_.robot.tool_geometry.local;
    body.is_robot = true;
    body.link_index = static_cast<int>(fk.link_frames.size());
    bodies.push_back(body);
  }

  if (!scene_.part.geometry.empty()) {
    WorldBody body;
    body.name = scene_.part.name;
    body.geometry = scene_.part.geometry;
    body.pose = scene_.part.pose * scene_.part.geometry.local;
    bodies.push_back(body);
  }

  for (const Obstacle& obstacle : scene_.obstacles) {
    if (obstacle.geometry.empty()) {
      continue;
    }
    WorldBody body;
    body.name = obstacle.name;
    body.geometry = obstacle.geometry;
    body.pose = obstacle.pose * obstacle.geometry.local;
    bodies.push_back(body);
  }

  return bodies;
}

CollisionResult CollisionChecker::evaluate(const std::vector<WorldBody>& bodies) const {
  CollisionResult result;
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    for (std::size_t j = i + 1; j < bodies.size(); ++j) {
      const WorldBody& a = bodies[i];
      const WorldBody& b = bodies[j];
      if (!a.is_robot && !b.is_robot) {
        continue;
      }
      if (a.is_robot && b.is_robot) {
        if (!options_.check_self) {
          continue;
        }
        if (std::abs(a.link_index - b.link_index) <= 1) {
          continue;
        }
      }
      const double distance = geometryDistance(a.geometry, a.pose, b.geometry, b.pose);
      if (distance < result.min_distance) {
        result.min_distance = distance;
        result.body_a = a.name;
        result.body_b = b.name;
      }
      if (distance < options_.margin) {
        result.collision = true;
      }
    }
  }

  if (options_.check_ground) {
    for (const WorldBody& body : bodies) {
      if (!body.is_robot) {
        continue;
      }
      const double distance = geometryGroundDistance(body.geometry, body.pose, options_.ground_z);
      if (distance < result.min_distance) {
        result.min_distance = distance;
        result.body_a = body.name;
        result.body_b = "ground";
      }
      if (distance < options_.margin) {
        result.collision = true;
      }
    }
  }

  return result;
}

bool isFclAvailable() {
#ifdef RP_WITH_FCL
  return true;
#else
  return false;
#endif
}

CollisionResult CollisionChecker::evaluateFcl(const std::vector<WorldBody>& bodies) const {
  CollisionResult result;
#ifdef RP_WITH_FCL
  std::vector<FclBody> fcl_bodies;
  fcl_bodies.reserve(bodies.size() + 1);
  for (const WorldBody& body : bodies) {
    FclBody fcl_body;
    fcl_body.name = body.name;
    fcl_body.transform = toFclTransform(body.pose);
    fcl_body.geometry = makeFclGeometry(body.geometry, &fcl_body.transform);
    if (fcl_body.geometry == nullptr) {
      continue;
    }
    fcl_body.is_robot = body.is_robot;
    fcl_body.link_index = body.link_index;
    fcl_bodies.push_back(std::move(fcl_body));
  }

  if (options_.check_ground) {
    FclBody ground;
    ground.name = "ground";
    ground.geometry = std::make_shared<fcl::Boxd>(20.0, 20.0, 0.2);
    ground.transform.translation() = fcl::Vector3d(0.0, 0.0, options_.ground_z - 0.1);
    ground.is_robot = false;
    ground.link_index = -1;
    fcl_bodies.push_back(std::move(ground));
  }

  for (std::size_t i = 0; i < fcl_bodies.size(); ++i) {
    for (std::size_t j = i + 1; j < fcl_bodies.size(); ++j) {
      const FclBody& a = fcl_bodies[i];
      const FclBody& b = fcl_bodies[j];
      if (!a.is_robot && !b.is_robot) {
        continue;
      }
      if (a.is_robot && b.is_robot) {
        if (!options_.check_self) {
          continue;
        }
        if (std::abs(a.link_index - b.link_index) <= 1) {
          continue;
        }
      }

      fcl::CollisionRequestd collision_request;
      collision_request.num_max_contacts = 1;
      fcl::CollisionResultd collision_result;
      const std::size_t contacts =
          fcl::collide(a.geometry.get(), a.transform, b.geometry.get(), b.transform,
                       collision_request, collision_result);

      double distance = 0.0;
      bool collides_now = contacts > 0;
      if (!collides_now) {
        fcl::DistanceRequestd distance_request;
        fcl::DistanceResultd distance_result;
        distance = fcl::distance(a.geometry.get(), a.transform, b.geometry.get(), b.transform,
                                 distance_request, distance_result);
        if (!std::isfinite(distance) || distance < 0.0) {
          distance = 1e9;
        }
        collides_now = distance < options_.margin;
      }

      if (distance < result.min_distance) {
        result.min_distance = distance;
        result.body_a = a.name;
        result.body_b = b.name;
      }
      if (collides_now) {
        result.collision = true;
      }
    }
  }
#else
  (void)bodies;
  result.collision = true;
  result.min_distance = -1.0;
  result.body_a = "fcl_unavailable";
  result.body_b = scene_.robot.name;
#endif
  return result;
}

CollisionResult CollisionChecker::checkConfiguration(const VecX& q) const {
  if (!Kinematics::isWithinLimits(scene_.robot, q)) {
    CollisionResult result;
    result.collision = true;
    result.min_distance = -1.0;
    result.body_a = "joint_limits";
    result.body_b = scene_.robot.name;
    return result;
  }
  const std::vector<WorldBody> bodies = buildBodies(q);
  if (options_.backend == CollisionBackend::Fcl && isFclAvailable()) {
    return evaluateFcl(bodies);
  }
  return evaluate(bodies);
}

bool CollisionChecker::collides(const VecX& q) const { return checkConfiguration(q).collision; }

double CollisionChecker::minimumDistance(const VecX& q) const {
  return checkConfiguration(q).min_distance;
}

CollisionResult CollisionChecker::checkTransition(const VecX& q0, const VecX& q1,
                                                  double max_joint_step) const {
  const int n = static_cast<int>(q0.size());
  double max_delta = 0.0;
  for (int i = 0; i < n && i < q1.size(); ++i) {
    max_delta = std::max(max_delta, std::abs(q1[i] - q0[i]));
  }
  const double step = std::max(1e-4, max_joint_step);
  const int samples = std::max(1, static_cast<int>(std::ceil(max_delta / step)));
  CollisionResult best;
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const VecX q = q0 + (q1 - q0) * t;
    const CollisionResult result = checkConfiguration(q);
    if (result.min_distance < best.min_distance) {
      best = result;
    }
    if (result.collision) {
      return result;
    }
  }
  return best;
}

}  // namespace rp
