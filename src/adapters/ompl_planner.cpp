#include "rp/adapters/motion_planner.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#ifdef RP_WITH_OMPL
#include <ompl/base/MotionValidator.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
#endif

namespace rp {

namespace {

std::string lowerCase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
  return value;
}

#ifdef RP_WITH_OMPL

using RealVectorSpace = ompl::base::RealVectorStateSpace;

VecX stateToVector(const ompl::base::State* state, Eigen::Index dimension) {
  const auto* values = state->as<RealVectorSpace::StateType>();
  VecX q(dimension);
  for (Eigen::Index i = 0; i < dimension; ++i) {
    q[i] = values->values[i];
  }
  return q;
}

void vectorToState(const VecX& q, ompl::base::State* state) {
  auto* values = state->as<RealVectorSpace::StateType>();
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    values->values[i] = q[i];
  }
}

class ChainMotionValidator : public ompl::base::MotionValidator {
 public:
  ChainMotionValidator(const ompl::base::SpaceInformationPtr& space_information,
                       const CollisionChecker& checker, double collision_step)
      : ompl::base::MotionValidator(space_information),
        checker_(checker),
        collision_step_(std::max(1e-4, collision_step)),
        dimension_(static_cast<Eigen::Index>(space_information->getStateDimension())) {}

  bool checkMotion(const ompl::base::State* start, const ompl::base::State* goal) const override {
    const VecX q0 = stateToVector(start, dimension_);
    const VecX q1 = stateToVector(goal, dimension_);
    if (!checker_.checkTransition(q0, q1, collision_step_).collision) {
      ++valid_;
      return true;
    }
    ++invalid_;
    return false;
  }

  bool checkMotion(const ompl::base::State* start, const ompl::base::State* goal,
                   std::pair<ompl::base::State*, double>& last_valid) const override {
    const VecX q0 = stateToVector(start, dimension_);
    const VecX q1 = stateToVector(goal, dimension_);
    if (!checker_.checkTransition(q0, q1, collision_step_).collision) {
      ++valid_;
      return true;
    }
    ++invalid_;

    double low = 0.0;
    double high = 1.0;
    for (int iteration = 0; iteration < 12; ++iteration) {
      const double middle = 0.5 * (low + high);
      const VecX q = q0 + (q1 - q0) * middle;
      if (!checker_.checkTransition(q0, q, collision_step_).collision) {
        low = middle;
      } else {
        high = middle;
      }
    }
    last_valid.second = low;
    if (last_valid.first != nullptr) {
      vectorToState(q0 + (q1 - q0) * low, last_valid.first);
    }
    return false;
  }

 private:
  const CollisionChecker& checker_;
  double collision_step_;
  Eigen::Index dimension_;
};

ompl::base::PlannerPtr makePlanner(MotionPlannerBackend backend,
                                   const ompl::base::SpaceInformationPtr& space_information,
                                   double range) {
  switch (backend) {
    case MotionPlannerBackend::OmplRRTStar: {
      auto planner = std::make_shared<ompl::geometric::RRTstar>(space_information);
      if (range > 0.0) {
        planner->setRange(range);
      }
      return planner;
    }
    case MotionPlannerBackend::OmplBITstar:
      return std::make_shared<ompl::geometric::BITstar>(space_information);
    case MotionPlannerBackend::OmplPRM:
      return std::make_shared<ompl::geometric::PRM>(space_information);
    case MotionPlannerBackend::OmplEST:
      return std::make_shared<ompl::geometric::EST>(space_information);
    case MotionPlannerBackend::OmplKPIECE1:
      return std::make_shared<ompl::geometric::KPIECE1>(space_information);
    case MotionPlannerBackend::OmplRRTConnect:
    default: {
      auto planner = std::make_shared<ompl::geometric::RRTConnect>(space_information);
      if (range > 0.0) {
        planner->setRange(range);
      }
      return planner;
    }
  }
}

#endif  // RP_WITH_OMPL

}  // namespace

bool isOmplAvailable() {
#ifdef RP_WITH_OMPL
  return true;
#else
  return false;
#endif
}

std::string motionBackendName(MotionPlannerBackend backend) {
  switch (backend) {
    case MotionPlannerBackend::InternalRRTConnect:
      return "internal_rrtconnect";
    case MotionPlannerBackend::OmplRRTConnect:
      return "ompl_rrtconnect";
    case MotionPlannerBackend::OmplRRTStar:
      return "ompl_rrtstar";
    case MotionPlannerBackend::OmplBITstar:
      return "ompl_bitstar";
    case MotionPlannerBackend::OmplPRM:
      return "ompl_prm";
    case MotionPlannerBackend::OmplEST:
      return "ompl_est";
    case MotionPlannerBackend::OmplKPIECE1:
      return "ompl_kpiece1";
    case MotionPlannerBackend::TesseractOMPL:
      return "tesseract_ompl";
  }
  return "internal_rrtconnect";
}

bool parseMotionBackend(const std::string& name, MotionPlannerBackend* backend) {
  if (backend == nullptr) {
    return false;
  }
  const std::string normalized = lowerCase(name);
  if (normalized == "internal" || normalized == "internal_rrt" ||
      normalized == "internal_rrtconnect" || normalized == "rrtconnect") {
    *backend = MotionPlannerBackend::InternalRRTConnect;
    return true;
  }
  if (normalized == "ompl" || normalized == "ompl_rrt" || normalized == "ompl_rrtconnect" ||
      normalized == "rrt_connect") {
    *backend = MotionPlannerBackend::OmplRRTConnect;
    return true;
  }
  if (normalized == "ompl_rrtstar" || normalized == "rrtstar" || normalized == "rrt_star") {
    *backend = MotionPlannerBackend::OmplRRTStar;
    return true;
  }
  if (normalized == "ompl_bitstar" || normalized == "bitstar" || normalized == "bit_star") {
    *backend = MotionPlannerBackend::OmplBITstar;
    return true;
  }
  if (normalized == "ompl_prm" || normalized == "prm") {
    *backend = MotionPlannerBackend::OmplPRM;
    return true;
  }
  if (normalized == "ompl_est" || normalized == "est") {
    *backend = MotionPlannerBackend::OmplEST;
    return true;
  }
  if (normalized == "ompl_kpiece1" || normalized == "kpiece1" || normalized == "kpiece") {
    *backend = MotionPlannerBackend::OmplKPIECE1;
    return true;
  }
  if (normalized == "tesseract" || normalized == "tesseract_ompl") {
    *backend = MotionPlannerBackend::TesseractOMPL;
    return true;
  }
  return false;
}

MotionPlannerResult planOmplPath(MotionPlannerBackend backend, const SerialChain& chain,
                                 const CollisionChecker& checker, const VecX& start,
                                 const VecX& goal, const MotionPlannerOptions& options) {
  MotionPlannerResult result;
  const auto begin = std::chrono::steady_clock::now();
  auto finish = [&]() {
    const auto end = std::chrono::steady_clock::now();
    result.planning_time = std::chrono::duration<double>(end - begin).count();
  };

  if (!isOmplBackend(backend)) {
    result.message = "not an OMPL backend";
    finish();
    return result;
  }

#ifdef RP_WITH_OMPL
  const Eigen::Index dimension = static_cast<Eigen::Index>(chain.joints.size());
  if (start.size() != dimension || goal.size() != dimension || dimension <= 0) {
    result.message = "start/goal dimension does not match the robot chain";
    finish();
    return result;
  }

  static std::once_flag logging_once;
  std::call_once(logging_once, []() { ompl::msg::setLogLevel(ompl::msg::LOG_WARN); });
  static std::once_flag seed_once;
  const unsigned int seed = options.seed;
  std::call_once(seed_once, [seed]() { ompl::RNG::setSeed(seed); });

  auto space = std::make_shared<RealVectorSpace>(static_cast<unsigned int>(dimension));
  ompl::base::RealVectorBounds bounds(static_cast<unsigned int>(dimension));
  for (Eigen::Index i = 0; i < dimension; ++i) {
    const Joint& joint = chain.joints[static_cast<std::size_t>(i)];
    bounds.setLow(static_cast<unsigned int>(i), joint.lower);
    bounds.setHigh(static_cast<unsigned int>(i), joint.upper);
  }
  space->setBounds(bounds);

  auto space_information = std::make_shared<ompl::base::SpaceInformation>(space);
  space_information->setStateValidityChecker(
      [&checker, dimension](const ompl::base::State* state) {
        return !checker.checkConfiguration(stateToVector(state, dimension)).collision;
      });
  space_information->setMotionValidator(std::make_shared<ChainMotionValidator>(
      space_information, checker, options.collision_step));
  space_information->setup();

  auto problem = std::make_shared<ompl::base::ProblemDefinition>(space_information);
  ompl::base::ScopedState<RealVectorSpace> start_state(space);
  ompl::base::ScopedState<RealVectorSpace> goal_state(space);
  for (Eigen::Index i = 0; i < dimension; ++i) {
    start_state->values[i] = start[i];
    goal_state->values[i] = goal[i];
  }
  problem->setStartAndGoalStates(start_state.get(), goal_state.get(), 1e-6);

  ompl::base::PlannerPtr planner = makePlanner(backend, space_information, options.range);
  planner->setProblemDefinition(problem);
  planner->setup();

  const ompl::base::PlannerStatus status = planner->solve(std::max(0.01, options.time_limit));
  if (!status) {
    result.message = "OMPL planner did not find a solution";
    finish();
    return result;
  }

  auto* path = problem->getSolutionPath()->as<ompl::geometric::PathGeometric>();
  if (path == nullptr) {
    result.message = "OMPL returned a non-geometric solution path";
    finish();
    return result;
  }

  if (options.simplify && path->getStateCount() > 2) {
    ompl::geometric::PathSimplifier simplifier(space_information);
    simplifier.simplifyMax(*path);
  }

  result.path.reserve(path->getStates().size());
  for (const ompl::base::State* state : path->getStates()) {
    result.path.push_back(stateToVector(state, dimension));
  }
  if (result.path.size() < 2) {
    result.message = "OMPL returned an empty path";
    finish();
    return result;
  }
  if ((result.path.front() - start).norm() > 1e-5 || (result.path.back() - goal).norm() > 1e-5) {
    result.message = "OMPL path endpoints do not match the request";
    finish();
    return result;
  }

  result.success = true;
  result.message = motionBackendName(backend) + " solution";
  finish();
  return result;
#else
  (void)chain;
  (void)checker;
  (void)start;
  (void)goal;
  (void)options;
  result.message = "OMPL support was not enabled at build time";
  finish();
  return result;
#endif
}

}  // namespace rp
