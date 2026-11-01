#pragma once

#include "rp/core/scene_model/scene.h"
#include "rp/core/types.h"

#include <string>
#include <vector>

namespace rp {

enum class CollisionBackend { Internal, Fcl };

inline const char* collisionBackendName(CollisionBackend backend) {
  return backend == CollisionBackend::Fcl ? "fcl" : "internal";
}

struct CollisionOptions {
  CollisionBackend backend = CollisionBackend::Internal;
  double margin = 0.0;
  bool check_ground = true;
  double ground_z = 0.0;
  bool check_self = false;
};

bool isFclAvailable();

struct CollisionResult {
  bool collision = false;
  double min_distance = 1e9;
  std::string body_a;
  std::string body_b;
};

class CollisionChecker {
 public:
  explicit CollisionChecker(const SceneModel& scene);
  CollisionChecker(const SceneModel& scene, const CollisionOptions& options);

  CollisionResult checkConfiguration(const VecX& q) const;
  CollisionResult checkTransition(const VecX& q0, const VecX& q1, double max_joint_step) const;
  bool collides(const VecX& q) const;
  double minimumDistance(const VecX& q) const;

  const CollisionOptions& options() const { return options_; }

 private:
  struct WorldBody {
    std::string name;
    Geometry geometry;
    Isometry pose;
    bool is_robot = false;
    int link_index = -1;
  };

  std::vector<WorldBody> buildBodies(const VecX& q) const;
  CollisionResult evaluate(const std::vector<WorldBody>& bodies) const;
  CollisionResult evaluateFcl(const std::vector<WorldBody>& bodies) const;

  SceneModel scene_;
  CollisionOptions options_;
};

double geometryDistance(const Geometry& a, const Isometry& pose_a, const Geometry& b,
                        const Isometry& pose_b);
double geometryGroundDistance(const Geometry& geometry, const Isometry& pose, double ground_z);

}  // namespace rp
