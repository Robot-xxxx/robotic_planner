#include "rp/planning/planner.h"

#include "rp/adapters/kdl_kinematics.h"
#include "rp/adapters/tesseract_planner.h"
#include "rp/core/kinematics/kinematics.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <utility>

namespace rp {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

struct SampleEval {
  ScanSample sample;
  VecX q;
};

struct RunData {
  int strip_id = 0;
  int run_in_strip = 0;
  bool reversed = false;
  std::vector<SampleEval> samples;

  VecX startQ() const { return reversed ? samples.back().q : samples.front().q; }
  VecX endQ() const { return reversed ? samples.front().q : samples.back().q; }
};

struct TransitionData {
  bool valid = false;
  bool via_home = false;
  double duration = 0.0;
  double length_m = 0.0;
  std::vector<PlannedWaypoint> waypoints;
};

double maxAbsDifference(const VecX& a, const VecX& b) {
  if (a.size() != b.size()) {
    return kInfinity;
  }
  double result = 0.0;
  for (Eigen::Index i = 0; i < a.size(); ++i) {
    result = std::max(result, std::abs(a[i] - b[i]));
  }
  return result;
}

double jointVelocityTime(const SerialChain& chain, const VecX& q0, const VecX& q1) {
  double duration = 0.0;
  for (Eigen::Index i = 0; i < q0.size() && i < q1.size(); ++i) {
    const Joint& joint = chain.joints[static_cast<std::size_t>(i)];
    const double delta = std::abs(q1[i] - q0[i]);
    duration = std::max(duration, delta / std::max(1e-9, joint.velocity_limit));
  }
  return duration;
}

double jointAccelerationTime(const SerialChain& chain, const VecX& q0, const VecX& q1) {
  double duration = 0.0;
  for (Eigen::Index i = 0; i < q0.size() && i < q1.size(); ++i) {
    const Joint& joint = chain.joints[static_cast<std::size_t>(i)];
    const double delta = std::abs(q1[i] - q0[i]);
    if (delta < 1e-12) {
      continue;
    }
    const double vmax = std::max(1e-9, joint.velocity_limit);
    const double amax = std::max(1e-9, joint.acceleration_limit);
    double time = 0.0;
    if (delta <= vmax * vmax / amax) {
      time = 2.0 * std::sqrt(delta / amax);
    } else {
      time = delta / vmax + vmax / amax;
    }
    duration = std::max(duration, time);
  }
  return duration;
}

void accumulateExternalMotion(const SerialChain& chain, const std::vector<PlannedWaypoint>& waypoints,
                              double& revolute_rad, double& prismatic_m) {
  for (std::size_t k = 1; k < waypoints.size(); ++k) {
    const VecX& q0 = waypoints[k - 1].q;
    const VecX& q1 = waypoints[k].q;
    for (Eigen::Index i = 0; i < q0.size() && i < q1.size(); ++i) {
      const Joint& joint = chain.joints[static_cast<std::size_t>(i)];
      if (!joint.external) {
        continue;
      }
      const double delta = std::abs(q1[i] - q0[i]);
      if (joint.type == JointType::Revolute) {
        revolute_rad += delta;
      } else {
        prismatic_m += delta;
      }
    }
  }
}

std::vector<VecX> buildSeeds(const SerialChain& chain, const VecX* previous, int attempts) {
  std::vector<VecX> seeds;
  if (previous != nullptr) {
    seeds.push_back(*previous);
  }
  const std::vector<VecX> defaults = Kinematics::defaultSeeds(chain);
  for (const VecX& seed : defaults) {
    if (static_cast<int>(seeds.size()) >= attempts) {
      break;
    }
    seeds.push_back(seed);
  }
  return seeds;
}

bool solveSample(const SerialChain& chain, const CollisionChecker& checker,
                 const Isometry& target, const VecX& home, const VecX* previous,
                 const PlanningOptions& options, const KdlChainSolver* kdl_solver,
                 VecX* solution) {
  const std::vector<VecX> seeds =
      buildSeeds(chain, previous, std::max(1, options.ik_seed_attempts));
  IKOptions ik_options;
  ik_options.max_iterations = options.ik_max_iterations;

  bool found = false;
  double best_score = kInfinity;
  VecX best_q;

  for (const VecX& seed : seeds) {
    const IKResult ik = (kdl_solver != nullptr)
                            ? kdl_solver->solve(target, seed, ik_options)
                            : Kinematics::solve(chain, target, seed, ik_options);
    if (!ik.success || !Kinematics::isWithinLimits(chain, ik.q)) {
      continue;
    }
    if (checker.checkConfiguration(ik.q).collision) {
      continue;
    }

    double step = 0.0;
    if (previous != nullptr) {
      step = maxAbsDifference(ik.q, *previous);
      if (step > options.max_scan_joint_step) {
        continue;
      }
    }

    const double score = (previous != nullptr) ? (ik.q - *previous).norm()
                                               : (ik.q - home).norm();
    if (!found || score < best_score) {
      found = true;
      best_score = score;
      best_q = ik.q;
    }
  }

  if (found) {
    *solution = best_q;
  }
  return found;
}

TransitionData makeTransition(const SerialChain& chain, const CollisionChecker& checker,
                              const VecX& q0, const VecX& q1, const PlanningOptions& options,
                              bool allow_via_home);

TransitionData makeDirectTransition(const SerialChain& chain, const CollisionChecker& checker,
                                    const VecX& q0, const VecX& q1,
                                    const PlanningOptions& options) {
  TransitionData result;
  if (q0.size() != q1.size() || !Kinematics::isWithinLimits(chain, q0) ||
      !Kinematics::isWithinLimits(chain, q1)) {
    return result;
  }

  const double max_delta = maxAbsDifference(q0, q1);
  const double step = std::max(1e-4, options.collision_step);
  const int sample_count = std::max(1, static_cast<int>(std::ceil(max_delta / step)));

  const CollisionResult collision = checker.checkTransition(q0, q1, step);
  if (collision.collision) {
    return result;
  }

  result.valid = true;
  result.duration = std::max(1e-6, jointAccelerationTime(chain, q0, q1) * 1.10);

  double length = 0.0;
  Vec3 previous_tcp = Kinematics::forward(chain, q0).tool.translation();
  for (int i = 0; i <= sample_count; ++i) {
    const double fraction = static_cast<double>(i) / static_cast<double>(sample_count);
    const VecX q = q0 + (q1 - q0) * fraction;
    const Vec3 tcp = Kinematics::forward(chain, q).tool.translation();
    PlannedWaypoint waypoint;
    waypoint.time = result.duration * fraction;
    waypoint.q = q;
    waypoint.tcp = tcp;
    if (i > 0) {
      length += (tcp - previous_tcp).norm();
    }
    previous_tcp = tcp;
    result.waypoints.push_back(std::move(waypoint));
  }
  result.length_m = length;
  return result;
}

struct RRTNode {
  VecX q;
  int parent = -1;
};

using RRTree = std::vector<RRTNode>;

int nearestNode(const RRTree& tree, const VecX& q) {
  int best = -1;
  double best_distance = kInfinity;
  for (std::size_t i = 0; i < tree.size(); ++i) {
    const double distance = (tree[i].q - q).squaredNorm();
    if (distance < best_distance) {
      best_distance = distance;
      best = static_cast<int>(i);
    }
  }
  return best;
}

VecX sampleConfiguration(const SerialChain& chain, std::mt19937& generator) {
  VecX q(static_cast<Eigen::Index>(chain.joints.size()));
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    const Joint& joint = chain.joints[static_cast<std::size_t>(i)];
    const double fraction = uniform(generator);
    q[i] = joint.lower + (joint.upper - joint.lower) * fraction;
  }
  return q;
}

int extendTree(RRTree& tree, const VecX& target, const SerialChain& chain,
               const CollisionChecker& checker, const PlanningOptions& options, double step) {
  if (tree.empty()) {
    return -1;
  }
  const int nearest = nearestNode(tree, target);
  if (nearest < 0) {
    return -1;
  }
  const VecX& from = tree[static_cast<std::size_t>(nearest)].q;
  const VecX delta = target - from;
  const double distance = delta.norm();
  if (distance < 1e-9) {
    return -1;
  }

  VecX candidate = (distance <= step) ? target : from + delta * (step / distance);
  candidate = Kinematics::clampToLimits(chain, candidate);
  if ((candidate - from).norm() < 1e-7) {
    return -1;
  }
  if (checker.checkTransition(from, candidate, options.collision_step).collision) {
    return -1;
  }

  tree.push_back({candidate, nearest});
  return static_cast<int>(tree.size()) - 1;
}

int connectTree(RRTree& tree, const VecX& target, const SerialChain& chain,
                const CollisionChecker& checker, const PlanningOptions& options, double step) {
  int last = -1;
  for (int iteration = 0; iteration < 64; ++iteration) {
    const int extended = extendTree(tree, target, chain, checker, options, step);
    if (extended < 0) {
      return -1;
    }
    last = extended;
    if ((tree[static_cast<std::size_t>(last)].q - target).norm() < 1e-6) {
      return last;
    }
  }
  return -1;
}

std::vector<VecX> reconstructPath(const RRTree& tree, int index) {
  std::vector<VecX> path;
  while (index >= 0) {
    path.push_back(tree[static_cast<std::size_t>(index)].q);
    index = tree[static_cast<std::size_t>(index)].parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<VecX> shortcutPath(const std::vector<VecX>& path, const CollisionChecker& checker,
                               const PlanningOptions& options) {
  if (path.size() < 3) {
    return path;
  }
  std::vector<VecX> result;
  result.push_back(path.front());
  std::size_t current = 0;
  while (current + 1 < path.size()) {
    std::size_t next = path.size() - 1;
    for (; next > current + 1; --next) {
      if (!checker.checkTransition(path[current], path[next], options.collision_step).collision) {
        break;
      }
    }
    result.push_back(path[next]);
    current = next;
  }
  return result;
}

std::vector<VecX> planRRTConnect(const SerialChain& chain, const CollisionChecker& checker,
                                 const VecX& q0, const VecX& q1,
                                 const PlanningOptions& options) {
  const double step = std::max(0.05, options.collision_step * 4.0);
  RRTree start_tree{{q0, -1}};
  RRTree goal_tree{{q1, -1}};
  std::mt19937 generator(0x5EEDu);
  std::uniform_real_distribution<double> bias(0.0, 1.0);
  bool swapped = false;

  const int max_iterations = 3000;
  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    const VecX random_target =
        (bias(generator) < 0.15) ? (swapped ? q0 : q1) : sampleConfiguration(chain, generator);

    const int extended = extendTree(start_tree, random_target, chain, checker, options, step);
    if (extended >= 0) {
      const int connected = connectTree(goal_tree, start_tree[static_cast<std::size_t>(extended)].q,
                                        chain, checker, options, step);
      if (connected >= 0) {
        const RRTree& root_start_tree = swapped ? goal_tree : start_tree;
        const RRTree& root_goal_tree = swapped ? start_tree : goal_tree;
        const int start_index = swapped ? connected : extended;
        const int goal_index = swapped ? extended : connected;

        const std::vector<VecX> start_part = reconstructPath(root_start_tree, start_index);
        std::vector<VecX> goal_part = reconstructPath(root_goal_tree, goal_index);
        std::reverse(goal_part.begin(), goal_part.end());

        std::vector<VecX> path = start_part;
        std::size_t offset = 1;
        if (!path.empty() && !goal_part.empty() &&
            (path.back() - goal_part.front()).norm() > 1e-6) {
          offset = 0;
        }
        for (std::size_t i = offset; i < goal_part.size(); ++i) {
          path.push_back(goal_part[i]);
        }
        if (path.size() >= 2) {
          return shortcutPath(path, checker, options);
        }
      }
    }

    std::swap(start_tree, goal_tree);
    swapped = !swapped;
  }
  return {};
}

TransitionData buildPathTransition(const SerialChain& chain, const CollisionChecker& checker,
                                   const VecX& q0, const std::vector<VecX>& path,
                                   const PlanningOptions& options) {
  (void)checker;
  (void)options;
  TransitionData result;
  if (path.size() < 2) {
    return result;
  }

  result.valid = true;
  double time = 0.0;
  Vec3 previous_tcp = Kinematics::forward(chain, path.front()).tool.translation();
  PlannedWaypoint first;
  first.time = 0.0;
  first.q = path.front();
  first.tcp = previous_tcp;
  result.waypoints.push_back(std::move(first));

  for (std::size_t i = 1; i < path.size(); ++i) {
    const double segment_time =
        std::max(1e-6, jointAccelerationTime(chain, path[i - 1], path[i]) * 1.10);
    time += segment_time;
    const Vec3 tcp = Kinematics::forward(chain, path[i]).tool.translation();
    result.length_m += (tcp - previous_tcp).norm();
    previous_tcp = tcp;
    PlannedWaypoint waypoint;
    waypoint.time = time;
    waypoint.q = path[i];
    waypoint.tcp = tcp;
    result.waypoints.push_back(std::move(waypoint));
  }
  result.duration = time;
  (void)q0;
  return result;
}

TransitionData makeRRTTransition(const SerialChain& chain, const CollisionChecker& checker,
                                 const VecX& q0, const VecX& q1,
                                 const PlanningOptions& options) {
  TransitionData result;
  if (q0.size() != q1.size() || !Kinematics::isWithinLimits(chain, q0) ||
      !Kinematics::isWithinLimits(chain, q1)) {
    return result;
  }
  std::vector<VecX> path;
  if (isOmplBackend(options.motion_backend)) {
    MotionPlannerOptions planner_options;
    planner_options.time_limit = options.ompl_time_limit;
    planner_options.range = options.ompl_range;
    planner_options.seed = options.ompl_seed;
    planner_options.simplify = options.ompl_simplify;
    planner_options.collision_step = options.collision_step;
    MotionPlannerResult ompl =
        planOmplPath(options.motion_backend, chain, checker, q0, q1, planner_options);
    if (!ompl.success) {
      return result;
    }
    path = std::move(ompl.path);
  } else {
    path = planRRTConnect(chain, checker, q0, q1, options);
  }

  if (path.size() < 2) {
    return result;
  }
  if ((path.front() - q0).norm() > 1e-5 || (path.back() - q1).norm() > 1e-5) {
    return result;
  }
  return buildPathTransition(chain, checker, q0, path, options);
}

TransitionData makeTransition(const SerialChain& chain, const CollisionChecker& checker,
                              const VecX& q0, const VecX& q1, const PlanningOptions& options,
                              bool allow_via_home) {
  TransitionData direct = makeDirectTransition(chain, checker, q0, q1, options);
  if (direct.valid) {
    return direct;
  }

  TransitionData rrt = makeRRTTransition(chain, checker, q0, q1, options);
  if (rrt.valid) {
    return rrt;
  }

  if (!allow_via_home) {
    return direct;
  }

  const VecX home = homeConfiguration(chain);
  if (maxAbsDifference(q0, home) < 1e-9 || maxAbsDifference(q1, home) < 1e-9) {
    return direct;
  }

  const TransitionData first = makeTransition(chain, checker, q0, home, options, false);
  if (!first.valid) {
    return direct;
  }
  const TransitionData second = makeTransition(chain, checker, home, q1, options, false);
  if (!second.valid) {
    return direct;
  }

  TransitionData result;
  result.valid = true;
  result.via_home = true;
  result.duration = first.duration + second.duration;
  result.length_m = first.length_m + second.length_m;
  result.waypoints = first.waypoints;
  for (PlannedWaypoint waypoint : second.waypoints) {
    waypoint.time += first.duration;
    result.waypoints.push_back(std::move(waypoint));
  }
  return result;
}

TransitionData makeCostTransition(const SerialChain& chain, const CollisionChecker& checker,
                                  const VecX& q0, const VecX& q1, const PlanningOptions& options,
                                  bool allow_via_home, bool estimate_only) {
  if (!estimate_only) {
    return makeTransition(chain, checker, q0, q1, options, allow_via_home);
  }
  TransitionData estimate;
  if (q0.size() == q1.size() && Kinematics::isWithinLimits(chain, q0) &&
      Kinematics::isWithinLimits(chain, q1)) {
    estimate.valid = true;
    estimate.duration = std::max(1e-6, jointAccelerationTime(chain, q0, q1) * 1.10);
  }
  return estimate;
}

constexpr int kStartNode = -1;

class TransitionCache {
 public:
  TransitionCache(const SerialChain& chain, const CollisionChecker& checker,
                  const std::vector<RunData>& runs, const VecX& home,
                  const PlanningOptions& options, bool estimate_only)
      : chain_(chain),
        checker_(checker),
        runs_(runs),
        home_(home),
        options_(options),
        estimate_only_(estimate_only) {}

  const TransitionData& get(int from, int to) {
    const auto key = std::make_pair(from, to);
    const auto found = values_.find(key);
    if (found != values_.end()) {
      return found->second;
    }

    const VecX q0 = (from == kStartNode) ? home_ : runs_[static_cast<std::size_t>(from)].endQ();
    const VecX q1 = (to == kStartNode) ? home_ : runs_[static_cast<std::size_t>(to)].startQ();
    const bool allow_via_home = (from != kStartNode);
    const auto inserted = values_.emplace(
        key, makeCostTransition(chain_, checker_, q0, q1, options_, allow_via_home, estimate_only_));
    return inserted.first->second;
  }

 private:
  const SerialChain& chain_;
  const CollisionChecker& checker_;
  const std::vector<RunData>& runs_;
  const VecX& home_;
  const PlanningOptions& options_;
  bool estimate_only_ = false;
  std::map<std::pair<int, int>, TransitionData> values_;
};

double costOfOrder(const std::vector<int>& order, TransitionCache& cache) {
  double cost = 0.0;
  int previous = kStartNode;
  for (const int index : order) {
    const TransitionData& transition = cache.get(previous, index);
    if (!transition.valid) {
      return kInfinity;
    }
    cost += transition.duration;
    previous = index;
  }
  return cost;
}

void twoOpt(std::vector<int>& order, TransitionCache& cache) {
  double best_cost = costOfOrder(order, cache);
  if (!std::isfinite(best_cost)) {
    return;
  }
  for (int iteration = 0; iteration < 100; ++iteration) {
    bool improved = false;
    const int n = static_cast<int>(order.size());
    for (int i = 0; i < n - 1; ++i) {
      for (int j = i + 1; j < n; ++j) {
        std::vector<int> candidate = order;
        std::reverse(candidate.begin() + i, candidate.begin() + j + 1);
        const double candidate_cost = costOfOrder(candidate, cache);
        if (candidate_cost + 1e-9 < best_cost) {
          order = std::move(candidate);
          best_cost = candidate_cost;
          improved = true;
        }
      }
    }
    if (!improved) {
      break;
    }
  }
}

struct OrderResult {
  std::vector<int> order;
  bool valid = false;
};

OrderResult buildOrder(const SerialChain& chain, const CollisionChecker& checker,
                       std::vector<RunData>& runs, const VecX& home,
                       const PlanningOptions& options) {
  OrderResult result;
  const int n = static_cast<int>(runs.size());
  if (n == 0) {
    return result;
  }

  const bool estimate_only = isOmplBackend(options.motion_backend);
  std::vector<bool> used(static_cast<std::size_t>(n), false);
  VecX current = home;
  bool all_valid = true;

  for (int step = 0; step < n; ++step) {
    int best_index = -1;
    bool best_reversed = false;
    double best_cost = kInfinity;

    for (int index = 0; index < n; ++index) {
      if (used[static_cast<std::size_t>(index)]) {
        continue;
      }
      for (int orientation = 0; orientation < 2; ++orientation) {
        const bool reversed = orientation == 1;
        runs[static_cast<std::size_t>(index)].reversed = reversed;
        const VecX start = runs[static_cast<std::size_t>(index)].startQ();
        const bool allow_via_home = maxAbsDifference(current, home) > 1e-9;
        const TransitionData transition = makeCostTransition(
            chain, checker, current, start, options, allow_via_home, estimate_only);
        if (transition.valid && transition.duration < best_cost) {
          best_cost = transition.duration;
          best_index = index;
          best_reversed = reversed;
        }
      }
      runs[static_cast<std::size_t>(index)].reversed = false;
    }

    if (best_index < 0) {
      for (int index = 0; index < n; ++index) {
        if (!used[static_cast<std::size_t>(index)]) {
          best_index = index;
          best_reversed = false;
          break;
        }
      }
      all_valid = false;
    }

    runs[static_cast<std::size_t>(best_index)].reversed = best_reversed;
    used[static_cast<std::size_t>(best_index)] = true;
    result.order.push_back(best_index);
    current = runs[static_cast<std::size_t>(best_index)].endQ();
  }

  result.valid = all_valid;

  auto freshCache = [&]() {
    return std::make_unique<TransitionCache>(chain, checker, runs, home, options, estimate_only);
  };

  if (options.optimize_order) {
    {
      auto cache = freshCache();
      twoOpt(result.order, *cache);
    }

    for (int pass = 0; pass < 4; ++pass) {
      auto cache = freshCache();
      bool flipped = false;

      for (int position = 0; position < n; ++position) {
        const int run_index = result.order[static_cast<std::size_t>(position)];
        RunData& run = runs[static_cast<std::size_t>(run_index)];
        const int previous = (position == 0) ? kStartNode
                                             : result.order[static_cast<std::size_t>(position - 1)];
        const int next = (position + 1 < n)
                             ? result.order[static_cast<std::size_t>(position + 1)]
                             : kStartNode;

        const VecX previous_end =
            (previous == kStartNode) ? home : runs[static_cast<std::size_t>(previous)].endQ();
        const VecX old_start = run.startQ();
        const VecX old_end = run.endQ();

        bool old_valid = true;
        double old_cost = 0.0;
        const TransitionData& incoming = cache->get(previous, run_index);
        if (!incoming.valid) {
          old_valid = false;
        } else {
          old_cost += incoming.duration;
        }
        if (next != kStartNode) {
          const TransitionData& outgoing = cache->get(run_index, next);
          if (!outgoing.valid) {
            old_valid = false;
          } else {
            old_cost += outgoing.duration;
          }
        }

        const bool allow_via_home = (previous != kStartNode);
        const TransitionData new_incoming = makeCostTransition(
            chain, checker, previous_end, old_end, options, allow_via_home, estimate_only);
        bool new_valid = new_incoming.valid;
        double new_cost = new_incoming.valid ? new_incoming.duration : kInfinity;
        if (next != kStartNode && new_valid) {
          const VecX next_start = runs[static_cast<std::size_t>(next)].startQ();
          const TransitionData new_outgoing = makeCostTransition(
              chain, checker, old_start, next_start, options, true, estimate_only);
          if (!new_outgoing.valid) {
            new_valid = false;
          } else {
            new_cost += new_outgoing.duration;
          }
        }

        if (new_valid && (!old_valid || new_cost + 1e-9 < old_cost)) {
          run.reversed = !run.reversed;
          flipped = true;
          break;
        }
      }

      if (!flipped) {
        break;
      }

      auto recache = freshCache();
      twoOpt(result.order, *recache);
    }
  }

  return result;
}

struct SampleCounters {
  int total = 0;
  int feasible = 0;
};

std::vector<RunData> buildRuns(const SerialChain& chain, const CollisionChecker& checker,
                               const std::vector<ScanStrip>& strips, const VecX& home,
                               const PlanningOptions& options, const KdlChainSolver* kdl_solver,
                               SampleCounters* counters) {
  std::vector<RunData> runs;

  for (const ScanStrip& strip : strips) {
    std::vector<SampleEval> pending;
    int run_in_strip = 0;
    VecX previous;
    bool has_previous = false;

    auto closeRun = [&]() {
      if (pending.size() >= 2) {
        RunData run;
        run.strip_id = strip.id;
        run.run_in_strip = run_in_strip++;
        run.samples = std::move(pending);
        runs.push_back(std::move(run));
      }
      pending.clear();
      has_previous = false;
    };

    for (const ScanSample& sample : strip.samples) {
      ++counters->total;
      VecX q;
      const VecX* previous_ptr = has_previous ? &previous : nullptr;
      if (!solveSample(chain, checker, sample.tool_pose, home, previous_ptr, options, kdl_solver,
                       &q)) {
        closeRun();
        continue;
      }
      ++counters->feasible;

      if (has_previous) {
        const double step = maxAbsDifference(previous, q);
        const bool collision =
            step > options.max_scan_joint_step ||
            checker.checkTransition(previous, q, options.collision_step).collision;
        if (collision) {
          closeRun();
        }
      }

      SampleEval evaluation;
      evaluation.sample = sample;
      evaluation.q = q;
      pending.push_back(std::move(evaluation));
      previous = q;
      has_previous = true;
    }

    closeRun();
  }

  return runs;
}

PlannedRun assembleRun(const SerialChain& chain, const RunData& run, int id, double start_time,
                       const ScannerParams& scanner, double& length_out) {
  PlannedRun planned;
  planned.id = id;
  planned.strip_id = run.strip_id;
  planned.reversed = run.reversed;
  planned.start_time = start_time;

  const std::size_t count = run.samples.size();
  double relative_time = 0.0;
  double length = 0.0;
  Vec3 previous_tcp = Vec3::Zero();

  for (std::size_t k = 0; k < count; ++k) {
    const std::size_t source = run.reversed ? (count - 1 - k) : k;
    const SampleEval& evaluation = run.samples[source];
    const Vec3 tcp = evaluation.sample.tool_pose.translation();

    if (k > 0) {
      const std::size_t previous_source = run.reversed ? (count - k) : (k - 1);
      const VecX& q_previous = run.samples[previous_source].q;
      const double distance = (tcp - previous_tcp).norm();
      const double joint_time = jointVelocityTime(chain, q_previous, evaluation.q);
      const double scan_time = scanner.scan_speed > 1e-9 ? distance / scanner.scan_speed : 0.0;
      relative_time += std::max({joint_time, scan_time, 1e-6});
      length += distance;
    }

    PlannedWaypoint waypoint;
    waypoint.time = start_time + relative_time;
    waypoint.q = evaluation.q;
    waypoint.tcp = tcp;
    planned.waypoints.push_back(std::move(waypoint));
    previous_tcp = tcp;
  }

  planned.duration = relative_time;
  planned.end_time = planned.start_time + planned.duration;
  planned.length_m = length;
  length_out = length;
  return planned;
}

}  // namespace

PlanResult Planner::plan(const SceneModel& scene, const TargetSurface& surface,
                         const ScannerParams& scanner, const PlanningOptions& options) {
  PlanResult result;
  result.task_type = "surface_coverage";
  if (isTesseractBackend(options.motion_backend)) {
    result.message = "Tesseract backend currently supports pose_sequence tasks";
    return result;
  }
  std::string chain_error;
  if (!validateChain(scene.robot, &chain_error)) {
    result.message = "invalid robot chain: " + chain_error;
    return result;
  }
  if (surface.size_u <= 0.0 || surface.size_v <= 0.0) {
    result.message = "target surface has non-positive size";
    return result;
  }
  if (scanner.scan_speed <= 0.0 || scanner.sample_spacing <= 0.0 || scanner.fov_width <= 0.0) {
    result.message = "scanner parameters are invalid";
    return result;
  }

  if (options.collision_backend == CollisionBackend::Fcl && !isFclAvailable()) {
    result.message = "FCL collision backend is not available in this build";
    return result;
  }

  CollisionOptions collision_options;
  collision_options.backend = options.collision_backend;
  collision_options.margin = options.collision_margin;
  collision_options.check_ground = scene.has_ground;
  collision_options.ground_z = scene.ground_z;
  CollisionChecker checker(scene, collision_options);

  const VecX home = homeConfiguration(scene.robot);
  if (!Kinematics::isWithinLimits(scene.robot, home)) {
    result.message = "home configuration violates joint limits";
    return result;
  }

  std::unique_ptr<KdlChainSolver> kdl_solver;
  if (options.ik_backend == IKBackend::Kdl) {
    if (!isKdlAvailable()) {
      result.message = "KDL IK backend is not available in this build";
      return result;
    }
    kdl_solver = std::make_unique<KdlChainSolver>(scene.robot);
    if (!kdl_solver->valid()) {
      result.message = "failed to build KDL chain: " + kdl_solver->error();
      return result;
    }
  }

  const std::vector<ScanStrip> strips =
      CoverageGenerator::generateStrips(surface, scanner, options.sweep_along_u);
  result.metrics.strips_generated = static_cast<int>(strips.size());
  if (strips.empty()) {
    result.message = "no scan strips were generated";
    return result;
  }

  SampleCounters counters;
  std::vector<RunData> runs =
      buildRuns(scene.robot, checker, strips, home, options, kdl_solver.get(), &counters);
  result.metrics.samples_total = counters.total;
  result.metrics.samples_feasible = counters.feasible;

  const double surface_area = surface.size_u * surface.size_v;
  result.metrics.target_area_m2 = surface_area;

  CoverageGrid coverage(surface, options.coverage_cell);
  const bool sweep_along_u = options.sweep_along_u;
  const double sweep_size = sweep_along_u ? surface.size_u : surface.size_v;
  const int samples_per_strip =
      std::max(2, static_cast<int>(std::round(sweep_size / scanner.sample_spacing)));
  const double half_along = 0.5 * sweep_size / static_cast<double>(samples_per_strip);
  const double half_cross = 0.5 * scanner.fov_width;
  const double half_u = sweep_along_u ? half_along : half_cross;
  const double half_v = sweep_along_u ? half_cross : half_along;

  for (const RunData& run : runs) {
    for (const SampleEval& evaluation : run.samples) {
      coverage.addFootprint(evaluation.sample.u, evaluation.sample.v, half_u, half_v);
    }
  }
  result.metrics.covered_area_m2 = coverage.coveredArea();
  result.metrics.coverage_ratio = coverage.ratio();

  if (runs.empty()) {
    result.message = "no collision-free scan runs were found";
    result.metrics.full_coverage = result.metrics.coverage_ratio >= 0.999;
    return result;
  }

  const OrderResult order = buildOrder(scene.robot, checker, runs, home, options);
  result.metrics.order_valid = order.valid;

  const int n = static_cast<int>(runs.size());
  double time = 0.0;
  VecX previous_q = home;
  bool all_transitions_valid = true;
  bool collision_free = true;

  for (int position = 0; position < n; ++position) {
    const int run_index = order.order[static_cast<std::size_t>(position)];
    const RunData& run = runs[static_cast<std::size_t>(run_index)];

    const bool allow_via_home = (position != 0);
    const TransitionData transition =
        makeTransition(scene.robot, checker, previous_q, run.startQ(), options, allow_via_home);

    PlannedTransition planned_transition;
    planned_transition.from_run = (position == 0) ? -1 : order.order[static_cast<std::size_t>(position - 1)];
    planned_transition.to_run = run_index;
    planned_transition.start_time = time;
    if (transition.valid) {
      planned_transition.duration = transition.duration;
      planned_transition.length_m = transition.length_m;
      for (PlannedWaypoint waypoint : transition.waypoints) {
        waypoint.time += time;
        planned_transition.waypoints.push_back(std::move(waypoint));
      }
      time += transition.duration;
      result.metrics.transfer_time_s += transition.duration;
      result.metrics.tcp_path_m += transition.length_m;
      accumulateExternalMotion(scene.robot, planned_transition.waypoints,
                               result.metrics.external_revolute_rad,
                               result.metrics.external_prismatic_m);
    } else {
      all_transitions_valid = false;
      collision_free = false;
      result.warnings.push_back("no collision-free transfer to run " + std::to_string(run_index));
    }
    planned_transition.end_time = time;
    result.transitions.push_back(std::move(planned_transition));

    double run_length = 0.0;
    PlannedRun planned_run =
        assembleRun(scene.robot, run, static_cast<int>(result.runs.size()), time, scanner, run_length);
    time = planned_run.end_time;

    result.metrics.scan_time_s += planned_run.duration;
    result.metrics.tcp_path_m += planned_run.length_m;
    accumulateExternalMotion(scene.robot, planned_run.waypoints,
                             result.metrics.external_revolute_rad,
                             result.metrics.external_prismatic_m);

    for (const PlannedWaypoint& waypoint : planned_run.waypoints) {
      if (checker.checkConfiguration(waypoint.q).collision) {
        collision_free = false;
        result.warnings.push_back("run " + std::to_string(planned_run.id) +
                                  " contains a colliding waypoint");
        break;
      }
    }

    previous_q = run.endQ();
    result.runs.push_back(std::move(planned_run));
  }

  result.metrics.total_time_s = time;
  result.metrics.runs_planned = static_cast<int>(result.runs.size());
  result.metrics.full_coverage = result.metrics.coverage_ratio >= 0.999;
  result.metrics.collision_free = collision_free && all_transitions_valid;
  result.metrics.order_valid = order.valid && all_transitions_valid;

  if (counters.feasible < counters.total) {
    result.warnings.push_back(std::to_string(counters.total - counters.feasible) + " of " +
                              std::to_string(counters.total) +
                              " samples were rejected by IK or collision checks");
  }
  if (!order.valid) {
    result.warnings.push_back("no fully collision-free strip order was found");
  }

  result.success = !result.runs.empty() && result.metrics.order_valid &&
                   result.metrics.collision_free &&
                   (!options.require_full_coverage || result.metrics.full_coverage);

  std::ostringstream message;
  message << "coverage=" << result.metrics.coverage_ratio * 100.0 << "% runs="
          << result.metrics.runs_planned << " time=" << result.metrics.total_time_s
          << "s tcp_path=" << result.metrics.tcp_path_m << "m";
  result.message = message.str();
  return result;
}

PlanResult planPoseGroupsInternal(const SceneModel& scene,
                                  const std::vector<PoseGroup>& groups,
                                  const PlanningOptions& options,
                                  const std::string& task_type) {
  PlanResult result;
  result.task_type = task_type;

  std::string chain_error;
  if (!validateChain(scene.robot, &chain_error)) {
    result.message = "invalid robot chain: " + chain_error;
    return result;
  }
  if (groups.empty()) {
    result.message = "pose groups are empty";
    return result;
  }

  std::size_t total_poses = 0;
  for (const PoseGroup& group : groups) {
    total_poses += group.poses.size();
  }
  if (total_poses == 0) {
    result.message = "pose groups contain no poses";
    return result;
  }

  result.metrics.pose_groups_total = static_cast<int>(groups.size());
  result.metrics.poses_total = static_cast<int>(total_poses);
  result.metrics.samples_total = static_cast<int>(total_poses);

  if (options.collision_backend == CollisionBackend::Fcl && !isFclAvailable()) {
    result.message = "FCL collision backend is not available in this build";
    return result;
  }

  CollisionOptions collision_options;
  collision_options.backend = options.collision_backend;
  collision_options.margin = options.collision_margin;
  collision_options.check_ground = scene.has_ground;
  collision_options.ground_z = scene.ground_z;
  const CollisionChecker checker(scene, collision_options);

  const VecX home = homeConfiguration(scene.robot);
  if (!Kinematics::isWithinLimits(scene.robot, home)) {
    result.message = "home configuration violates joint limits";
    return result;
  }

  std::unique_ptr<KdlChainSolver> kdl_solver;
  if (options.ik_backend == IKBackend::Kdl) {
    if (!isKdlAvailable()) {
      result.message = "KDL IK backend is not available in this build";
      return result;
    }
    kdl_solver = std::make_unique<KdlChainSolver>(scene.robot);
    if (!kdl_solver->valid()) {
      result.message = "failed to build KDL chain: " + kdl_solver->error();
      return result;
    }
  }

  std::unique_ptr<TesseractPlanner> tesseract_planner;
  if (isTesseractBackend(options.motion_backend)) {
    if (!isTesseractAvailable()) {
      result.message = "Tesseract backend is not available in this build";
      return result;
    }
    tesseract_planner = std::make_unique<TesseractPlanner>(scene);
    if (!tesseract_planner->valid()) {
      result.message = "failed to initialize Tesseract: " + tesseract_planner->error();
      return result;
    }
  }

  auto planTransition = [&](const VecX& q0, const VecX& q1) -> TransitionData {
    if (tesseract_planner != nullptr) {
      std::vector<VecX> path;
      std::string planning_error;
      if (!tesseract_planner->planJointPath(q0, q1, options.ompl_time_limit, &path,
                                            &planning_error)) {
        return {};
      }
      if (path.size() < 2 || (path.front() - q0).norm() > 1e-4 ||
          (path.back() - q1).norm() > 1e-4) {
        return {};
      }
      for (std::size_t i = 1; i < path.size(); ++i) {
        if (checker.checkTransition(path[i - 1], path[i], options.collision_step).collision) {
          return {};
        }
      }
      return buildPathTransition(scene.robot, checker, q0, path, options);
    }
    return makeTransition(scene.robot, checker, q0, q1, options, true);
  };

  struct PoseEntry {
    int group_index = -1;
    int group_pose_index = -1;
    int input_index = -1;
    PoseTarget target;
    VecX q;
  };

  struct RuntimeGroup {
    int input_index = -1;
    std::string name;
    int poses_total = 0;
    bool optimize_order = true;
    std::vector<int> entry_indices;
  };

  std::vector<PoseEntry> entries;
  std::vector<RuntimeGroup> runtime_groups;
  runtime_groups.reserve(groups.size());
  int flat_input_index = 0;

  for (std::size_t group_index = 0; group_index < groups.size(); ++group_index) {
    const PoseGroup& group = groups[group_index];
    RuntimeGroup runtime_group;
    runtime_group.input_index = static_cast<int>(group_index);
    runtime_group.name =
        group.name.empty() ? ("group_" + std::to_string(group_index)) : group.name;
    runtime_group.poses_total = static_cast<int>(group.poses.size());
    runtime_group.optimize_order = group.optimize_order;

    for (std::size_t pose_index = 0; pose_index < group.poses.size(); ++pose_index) {
      VecX q;
      if (!solveSample(scene.robot, checker, group.poses[pose_index].pose, home, nullptr, options,
                       kdl_solver.get(), &q)) {
        result.warnings.push_back("group " + std::to_string(group_index) + " pose " +
                                  std::to_string(pose_index) + " (" +
                                  group.poses[pose_index].name +
                                  ") is unreachable or in collision");
        ++flat_input_index;
        continue;
      }
      PoseEntry entry;
      entry.group_index = static_cast<int>(group_index);
      entry.group_pose_index = static_cast<int>(pose_index);
      entry.input_index = flat_input_index++;
      entry.target = group.poses[pose_index];
      entry.q = q;
      runtime_group.entry_indices.push_back(static_cast<int>(entries.size()));
      entries.push_back(std::move(entry));
    }
    runtime_groups.push_back(std::move(runtime_group));
  }

  result.metrics.poses_reachable = static_cast<int>(entries.size());
  result.metrics.samples_feasible = static_cast<int>(entries.size());

  for (const RuntimeGroup& runtime_group : runtime_groups) {
    PlannedPoseGroup summary;
    summary.name = runtime_group.name;
    summary.input_index = runtime_group.input_index;
    summary.poses_total = runtime_group.poses_total;
    summary.optimize_order = runtime_group.optimize_order;
    result.pose_groups.push_back(std::move(summary));
  }

  if (entries.empty()) {
    result.message = "no pose in any group is reachable and collision free";
    result.metrics.collision_free = false;
    result.metrics.order_valid = false;
    return result;
  }

  std::vector<int> order;
  order.reserve(entries.size());
  VecX current_q = home;

  for (const RuntimeGroup& runtime_group : runtime_groups) {
    if (runtime_group.entry_indices.empty()) {
      continue;
    }
    std::vector<int> local_order = runtime_group.entry_indices;
    const bool optimize = options.optimize_order && runtime_group.optimize_order;
    if (optimize && local_order.size() > 1) {
      auto distanceCost = [&](const std::vector<int>& candidate, const VecX& start) {
        double cost = 0.0;
        VecX previous = start;
        for (const int index : candidate) {
          cost += (entries[static_cast<std::size_t>(index)].q - previous).norm();
          previous = entries[static_cast<std::size_t>(index)].q;
        }
        return cost;
      };

      std::vector<bool> used(local_order.size(), false);
      std::vector<int> greedy;
      greedy.reserve(local_order.size());
      VecX greedy_current = current_q;
      for (std::size_t step = 0; step < local_order.size(); ++step) {
        int best = -1;
        double best_distance = kInfinity;
        for (std::size_t i = 0; i < local_order.size(); ++i) {
          if (used[i]) {
            continue;
          }
          const double distance =
              (entries[static_cast<std::size_t>(local_order[i])].q - greedy_current)
                  .squaredNorm();
          if (distance < best_distance) {
            best_distance = distance;
            best = static_cast<int>(i);
          }
        }
        if (best < 0) {
          break;
        }
        used[static_cast<std::size_t>(best)] = true;
        greedy.push_back(local_order[static_cast<std::size_t>(best)]);
        greedy_current = entries[static_cast<std::size_t>(greedy.back())].q;
      }

      if (greedy.size() == local_order.size()) {
        double best_cost = distanceCost(greedy, current_q);
        for (int iteration = 0; iteration < 100; ++iteration) {
          bool improved = false;
          for (std::size_t i = 0; i + 1 < greedy.size(); ++i) {
            for (std::size_t j = i + 1; j < greedy.size(); ++j) {
              std::vector<int> candidate = greedy;
              std::reverse(candidate.begin() + static_cast<std::ptrdiff_t>(i),
                           candidate.begin() + static_cast<std::ptrdiff_t>(j) + 1);
              const double candidate_cost = distanceCost(candidate, current_q);
              if (candidate_cost + 1e-9 < best_cost) {
                greedy = std::move(candidate);
                best_cost = candidate_cost;
                improved = true;
              }
            }
          }
          if (!improved) {
            break;
          }
        }
        local_order = std::move(greedy);
      }
    }

    order.insert(order.end(), local_order.begin(), local_order.end());
    current_q = entries[static_cast<std::size_t>(local_order.back())].q;
  }

  double time = 0.0;
  VecX previous_q = home;
  bool transitions_valid = true;
  bool collision_free = true;
  int previous_input_index = -1;

  for (int position = 0; position < static_cast<int>(order.size()); ++position) {
    PoseEntry& entry = entries[static_cast<std::size_t>(order[static_cast<std::size_t>(position)])];

    TransitionData transition = planTransition(previous_q, entry.q);
    if (!transition.valid) {
      IKOptions ik_options;
      ik_options.max_iterations = options.ik_max_iterations;
      const std::vector<VecX> seeds =
          buildSeeds(scene.robot, &previous_q, std::max(1, options.ik_seed_attempts));
      bool found = false;
      for (const VecX& seed : seeds) {
        const IKResult ik = (kdl_solver != nullptr)
                                ? kdl_solver->solve(entry.target.pose, seed, ik_options)
                                : Kinematics::solve(scene.robot, entry.target.pose, seed, ik_options);
        if (!ik.success || !Kinematics::isWithinLimits(scene.robot, ik.q)) {
          continue;
        }
        if (checker.checkConfiguration(ik.q).collision) {
          continue;
        }
        const TransitionData candidate = planTransition(previous_q, ik.q);
        if (candidate.valid) {
          entry.q = ik.q;
          transition = candidate;
          found = true;
          break;
        }
      }
      if (!found) {
        transitions_valid = false;
        collision_free = false;
        result.warnings.push_back("no collision-free transfer to group " +
                                  std::to_string(entry.group_index) + " pose " +
                                  std::to_string(entry.group_pose_index));
        break;
      }
    }

    PlannedTransition planned_transition;
    planned_transition.from_pose = previous_input_index;
    planned_transition.to_pose = entry.input_index;
    planned_transition.start_time = time;
    planned_transition.duration = transition.duration;
    planned_transition.length_m = transition.length_m;
    for (PlannedWaypoint waypoint : transition.waypoints) {
      waypoint.time += time;
      planned_transition.waypoints.push_back(std::move(waypoint));
    }
    time += transition.duration;
    planned_transition.end_time = time;
    result.transitions.push_back(std::move(planned_transition));
    result.metrics.transfer_time_s += transition.duration;
    result.metrics.tcp_path_m += transition.length_m;
    accumulateExternalMotion(scene.robot, result.transitions.back().waypoints,
                             result.metrics.external_revolute_rad,
                             result.metrics.external_prismatic_m);

    const Isometry reached = Kinematics::forward(scene.robot, entry.q).tool;
    if (checker.checkConfiguration(entry.q).collision) {
      collision_free = false;
      result.warnings.push_back("group " + std::to_string(entry.group_index) + " pose " +
                                std::to_string(entry.group_pose_index) +
                                " is in collision at its final configuration");
    }

    PlannedPoseGroup& group_summary =
        result.pose_groups[static_cast<std::size_t>(entry.group_index)];
    if (group_summary.poses_visited == 0) {
      group_summary.start_time = time;
    }

    PlannedPoseVisit visit;
    visit.name = entry.target.name;
    visit.input_index = entry.input_index;
    visit.order = position;
    visit.group_index = entry.group_index;
    visit.group_pose_index = entry.group_pose_index;
    visit.group_name = runtime_groups[static_cast<std::size_t>(entry.group_index)].name;
    visit.q = entry.q;
    visit.tcp = reached.translation();
    visit.arrival_time = time;
    visit.dwell_time = std::max(0.0, entry.target.dwell_time);
    time += visit.dwell_time;
    group_summary.end_time = time;
    ++group_summary.poses_visited;
    result.metrics.dwell_time_s += visit.dwell_time;
    result.pose_visits.push_back(std::move(visit));

    previous_q = entry.q;
    previous_input_index = entry.input_index;
  }

  if (options.return_home && transitions_valid) {
    const TransitionData home_transition = planTransition(previous_q, home);
    if (!home_transition.valid) {
      transitions_valid = false;
      collision_free = false;
      result.warnings.push_back("no collision-free transfer back to home");
    } else {
      PlannedTransition planned_transition;
      planned_transition.from_pose = previous_input_index;
      planned_transition.to_pose = -1;
      planned_transition.start_time = time;
      planned_transition.duration = home_transition.duration;
      planned_transition.length_m = home_transition.length_m;
      for (PlannedWaypoint waypoint : home_transition.waypoints) {
        waypoint.time += time;
        planned_transition.waypoints.push_back(std::move(waypoint));
      }
      time += home_transition.duration;
      planned_transition.end_time = time;
      result.transitions.push_back(std::move(planned_transition));
      result.metrics.transfer_time_s += home_transition.duration;
      result.metrics.tcp_path_m += home_transition.length_m;
      accumulateExternalMotion(scene.robot, result.transitions.back().waypoints,
                               result.metrics.external_revolute_rad,
                               result.metrics.external_prismatic_m);
    }
  }

  for (const PlannedPoseGroup& group : result.pose_groups) {
    if (group.poses_visited > 0) {
      ++result.metrics.pose_groups_planned;
    }
  }

  result.metrics.total_time_s = time;
  result.metrics.runs_planned = static_cast<int>(result.pose_visits.size());
  result.metrics.order_valid = transitions_valid;
  result.metrics.collision_free = collision_free && transitions_valid;
  result.metrics.full_coverage = false;
  result.metrics.coverage_ratio = 0.0;

  result.success = !result.pose_visits.empty() && transitions_valid && collision_free &&
                   (!options.require_all_poses ||
                    result.metrics.poses_reachable == result.metrics.poses_total);

  std::ostringstream message;
  message << "groups=" << result.metrics.pose_groups_planned << "/"
          << result.metrics.pose_groups_total << " poses=" << result.metrics.poses_reachable
          << "/" << result.metrics.poses_total << " visits=" << result.pose_visits.size()
          << " time=" << result.metrics.total_time_s << "s tcp_path="
          << result.metrics.tcp_path_m << "m";
  result.message = message.str();
  return result;
}

PlanResult Planner::planPoseSequence(const SceneModel& scene,
                                     const std::vector<PoseTarget>& poses,
                                     const PlanningOptions& options) {
  PoseGroup group;
  group.name = "poses";
  group.poses = poses;
  group.optimize_order = true;
  return planPoseGroupsInternal(scene, {group}, options, "pose_sequence");
}

PlanResult Planner::planPoseGroups(const SceneModel& scene,
                                   const std::vector<PoseGroup>& groups,
                                   const PlanningOptions& options) {
  return planPoseGroupsInternal(scene, groups, options, "pose_groups");
}

}  // namespace rp
