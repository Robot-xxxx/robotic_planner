#pragma once

#include "rp/core/collision/collision.h"
#include "rp/core/scene_model/scene.h"
#include "rp/core/types.h"

#include <string>
#include <vector>

namespace rp {

enum class MotionPlannerBackend {
  InternalRRTConnect,
  OmplRRTConnect,
  OmplRRTStar,
  OmplBITstar,
  OmplPRM,
  OmplEST,
  OmplKPIECE1,
  TesseractOMPL
};

inline bool isOmplBackend(MotionPlannerBackend backend) {
  return backend >= MotionPlannerBackend::OmplRRTConnect &&
         backend <= MotionPlannerBackend::OmplKPIECE1;
}

inline bool isTesseractBackend(MotionPlannerBackend backend) {
  return backend == MotionPlannerBackend::TesseractOMPL;
}

struct MotionPlannerOptions {
  double time_limit = 0.5;
  double range = 0.0;
  unsigned int seed = 1;
  bool simplify = true;
  double collision_step = 0.05;
};

struct MotionPlannerResult {
  bool success = false;
  std::vector<VecX> path;
  double planning_time = 0.0;
  std::string message;
};

bool isOmplAvailable();
std::string motionBackendName(MotionPlannerBackend backend);
bool parseMotionBackend(const std::string& name, MotionPlannerBackend* backend);

MotionPlannerResult planOmplPath(MotionPlannerBackend backend, const SerialChain& chain,
                                 const CollisionChecker& checker, const VecX& start,
                                 const VecX& goal, const MotionPlannerOptions& options);

}  // namespace rp
