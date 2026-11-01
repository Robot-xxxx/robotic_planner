#include "rp/core/scene_model/scene.h"

#include <sstream>

namespace rp {

namespace {

bool finiteJoint(const Joint& joint) {
  return std::isfinite(joint.lower) && std::isfinite(joint.upper) && joint.lower <= joint.upper &&
         std::isfinite(joint.home) && std::isfinite(joint.velocity_limit) &&
         joint.velocity_limit > 0.0 && std::isfinite(joint.acceleration_limit) &&
         joint.acceleration_limit > 0.0;
}

}  // namespace

bool validateChain(const SerialChain& chain, std::string* error) {
  if (chain.joints.empty()) {
    if (error) {
      *error = "robot chain has no joints";
    }
    return false;
  }
  for (std::size_t i = 0; i < chain.joints.size(); ++i) {
    const Joint& joint = chain.joints[i];
    if (joint.name.empty()) {
      if (error) {
        *error = "joint " + std::to_string(i) + " has an empty name";
      }
      return false;
    }
    if (joint.axis.norm() < kEpsilon) {
      if (error) {
        *error = "joint " + joint.name + " has a zero axis";
      }
      return false;
    }
    if (!finiteJoint(joint)) {
      if (error) {
        *error = "joint " + joint.name + " has invalid limits or dynamic limits";
      }
      return false;
    }
  }
  if (!chain.link_geometries.empty() && chain.link_geometries.size() != chain.joints.size()) {
    if (error) {
      *error = "link_geometries size must be zero or equal to joint count";
    }
    return false;
  }
  if (error) {
    error->clear();
  }
  return true;
}

VecX homeConfiguration(const SerialChain& chain) {
  VecX q(static_cast<Eigen::Index>(chain.joints.size()));
  for (std::size_t i = 0; i < chain.joints.size(); ++i) {
    q[static_cast<Eigen::Index>(i)] = chain.joints[i].home;
  }
  return q;
}

}  // namespace rp
