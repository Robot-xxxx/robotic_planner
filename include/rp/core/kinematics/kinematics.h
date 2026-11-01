#pragma once

#include "rp/core/scene_model/scene.h"
#include "rp/core/types.h"

#include <string>
#include <vector>

namespace rp {

struct FKResult {
  std::vector<Isometry> link_frames;
  std::vector<Vec3> joint_origins;
  std::vector<Vec3> joint_axes;
  Isometry tool = Isometry::Identity();
};

struct IKOptions {
  double position_tolerance = 1e-4;
  double orientation_tolerance = 1e-3;
  int max_iterations = 200;
  double initial_damping = 0.05;
  double min_damping = 1e-6;
  double max_damping = 1e3;
  double max_step_norm = 0.25;
  int max_line_search = 4;
};

struct IKResult {
  bool success = false;
  VecX q;
  double position_error = 0.0;
  double orientation_error = 0.0;
  int iterations = 0;
};

class Kinematics {
 public:
  static FKResult forward(const SerialChain& chain, const VecX& q);
  static bool isWithinLimits(const SerialChain& chain, const VecX& q, double tolerance = 1e-9);
  static VecX clampToLimits(const SerialChain& chain, const VecX& q);

  static IKResult solve(const SerialChain& chain, const Isometry& target, const VecX& seed,
                        const IKOptions& options = IKOptions());

  static IKResult solveMultiSeed(const SerialChain& chain, const Isometry& target,
                                 const std::vector<VecX>& seeds,
                                 const IKOptions& options = IKOptions());

  static std::vector<VecX> defaultSeeds(const SerialChain& chain);
};

}  // namespace rp
