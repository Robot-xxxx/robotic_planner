#pragma once

#include "rp/core/scene_model/scene.h"
#include "rp/core/types.h"

#include <memory>
#include <string>
#include <vector>

namespace rp {

bool isTesseractAvailable();

class TesseractPlanner {
 public:
  explicit TesseractPlanner(const SceneModel& scene);
  ~TesseractPlanner();

  TesseractPlanner(TesseractPlanner&&) noexcept;
  TesseractPlanner& operator=(TesseractPlanner&&) noexcept;
  TesseractPlanner(const TesseractPlanner&) = delete;
  TesseractPlanner& operator=(const TesseractPlanner&) = delete;

  bool valid() const;
  const std::string& error() const;

  bool planJointPath(const VecX& start, const VecX& goal, double time_limit,
                     std::vector<VecX>* path, std::string* error) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rp
