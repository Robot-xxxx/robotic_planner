#pragma once

#include "rp/core/kinematics/kinematics.h"

#include <memory>
#include <string>

namespace rp {

bool isKdlAvailable();

class KdlChainSolver {
 public:
  explicit KdlChainSolver(const SerialChain& chain);
  ~KdlChainSolver();

  KdlChainSolver(KdlChainSolver&&) noexcept;
  KdlChainSolver& operator=(KdlChainSolver&&) noexcept;
  KdlChainSolver(const KdlChainSolver&) = delete;
  KdlChainSolver& operator=(const KdlChainSolver&) = delete;

  bool valid() const;
  const std::string& error() const;

  FKResult forward(const VecX& q) const;
  IKResult solve(const Isometry& target, const VecX& seed, const IKOptions& options) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rp
