#include "rp/adapters/tesseract_planner.h"

#include "rp/core/kinematics/kinematics.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#ifdef RP_WITH_TESSERACT
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/state_waypoint.h>
#include <tesseract/command_language/utils.h>
#include <tesseract/common/logging.h>
#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract/motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract/motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract/motion_planners/types.h>
#include <tesseract/geometry/impl/box.h>
#include <tesseract/geometry/impl/capsule.h>
#include <tesseract/geometry/impl/sphere.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>
#endif

namespace rp {

namespace {

std::string number(double value) {
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}

std::string vectorString(const Vec3& value) {
  return number(value.x()) + " " + number(value.y()) + " " + number(value.z());
}

Mat3 rotationFromZTo(const Vec3& direction) {
  const Vec3 normalized = direction.normalized();
  return Eigen::Quaterniond::FromTwoVectors(Vec3::UnitZ(), normalized).toRotationMatrix();
}

Vec3 rpyFromMatrix(const Mat3& rotation) {
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  double roll = 0.0;
  double yaw = 0.0;
  if (std::abs(std::cos(pitch)) > 1e-8) {
    roll = std::atan2(rotation(2, 1), rotation(2, 2));
    yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  } else {
    roll = 0.0;
    yaw = std::atan2(-rotation(0, 1), rotation(1, 1));
  }
  return Vec3(roll, pitch, yaw);
}

std::string originString(const Isometry& transform) {
  const Vec3 rpy = rpyFromMatrix(transform.rotation());
  return "xyz=\"" + vectorString(transform.translation()) + "\" rpy=\"" + vectorString(rpy) +
         "\"";
}

std::string inertialString() {
  return "<inertial><mass value=\"1.0\"/>"
         "<inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>"
         "</inertial>";
}

std::string collisionForGeometry(const Geometry& geometry, const Isometry& local) {
  if (geometry.empty()) {
    return {};
  }
  if (geometry.type == Geometry::Type::Capsule) {
    const Vec3 a = local * geometry.a;
    const Vec3 b = local * geometry.b;
    const Vec3 delta = b - a;
    const double length = delta.norm();
    if (length < 1e-9) {
      return {};
    }
    const Isometry capsule_pose = makeIsometry((a + b) * 0.5, rotationFromZTo(delta));
    return "<collision><origin " + originString(capsule_pose) +
           "/><geometry><tesseract:capsule radius=\"" + number(geometry.radius) +
           "\" length=\"" + number(length) + "\"/></geometry></collision>";
  }
  if (geometry.type == Geometry::Type::Sphere) {
    const Isometry pose = local * makeIsometry(geometry.center, Mat3::Identity());
    return "<collision><origin " + originString(pose) + "/><geometry><sphere radius=\"" +
           number(geometry.radius) + "\"/></geometry></collision>";
  }
  if (geometry.type == Geometry::Type::Box) {
    const Isometry pose = local * makeIsometry(geometry.center, Mat3::Identity());
    return "<collision><origin " + originString(pose) + "/><geometry><box size=\"" +
           number(2.0 * geometry.half_extents.x()) + " " +
           number(2.0 * geometry.half_extents.y()) + " " +
           number(2.0 * geometry.half_extents.z()) + "\"/></geometry></collision>";
  }
  if (geometry.type == Geometry::Type::Mesh && !geometry.vertices.empty()) {
    Vec3 minimum = geometry.vertices.front();
    Vec3 maximum = geometry.vertices.front();
    for (const Vec3& vertex : geometry.vertices) {
      minimum = minimum.cwiseMin(vertex);
      maximum = maximum.cwiseMax(vertex);
    }
    const Vec3 size = maximum - minimum;
    const Isometry pose = local * makeIsometry((minimum + maximum) * 0.5, Mat3::Identity());
    return "<collision><origin " + originString(pose) + "/><geometry><box size=\"" +
           number(std::max(1e-3, size.x())) + " " + number(std::max(1e-3, size.y())) + " " +
           number(std::max(1e-3, size.z())) + "\"/></geometry></collision>";
  }
  return {};
}

std::string makeUrdf(const SceneModel& scene) {
  const SerialChain& chain = scene.robot;
  std::ostringstream urdf;
  urdf << "<robot name=\"" << (chain.name.empty() ? "robot" : chain.name)
       << "\" tesseract:make_convex=\"true\">\n";
  urdf << "<link name=\"base_link\">" << inertialString() << "</link>\n";

  for (std::size_t i = 0; i < chain.joints.size(); ++i) {
    const Joint& joint = chain.joints[i];
    const std::string link_name = "link_" + std::to_string(i);
    urdf << "<link name=\"" << link_name << "\">" << inertialString();
    if (i < chain.link_geometries.size()) {
      urdf << collisionForGeometry(chain.link_geometries[i], chain.link_geometries[i].local);
    }
    urdf << "</link>\n";

    const Isometry origin =
        (i == 0) ? (chain.base * joint.origin) : joint.origin;
    urdf << "<joint name=\"" << joint.name << "\" type=\""
         << (joint.type == JointType::Revolute ? "revolute" : "prismatic") << "\">\n";
    urdf << "<parent link=\"" << (i == 0 ? "base_link" : "link_" + std::to_string(i - 1))
         << "\"/>\n";
    urdf << "<child link=\"" << link_name << "\"/>\n";
    urdf << "<origin " << originString(origin) << "/>\n";
    urdf << "<axis xyz=\"" << vectorString(joint.axis.normalized()) << "\"/>\n";
    urdf << "<limit lower=\"" << number(joint.lower) << "\" upper=\"" << number(joint.upper)
         << "\" velocity=\"" << number(joint.velocity_limit) << "\" effort=\"100\"/>\n";
    urdf << "</joint>\n";
  }

  urdf << "<link name=\"tool0\">" << inertialString();
  if (!chain.tool_geometry.empty()) {
    urdf << collisionForGeometry(chain.tool_geometry, chain.tool_geometry.local);
  }
  urdf << "</link>\n";
  urdf << "<joint name=\"tool_joint\" type=\"fixed\">\n";
  urdf << "<parent link=\""
       << (chain.joints.empty() ? "base_link"
                                : "link_" + std::to_string(chain.joints.size() - 1))
       << "\"/>\n";
  urdf << "<child link=\"tool0\"/>\n";
  urdf << "<origin " << originString(chain.tool) << "/>\n";
  urdf << "</joint>\n";

  if (!scene.part.geometry.empty()) {
    urdf << "<link name=\"part\">" << inertialString()
         << collisionForGeometry(scene.part.geometry, scene.part.geometry.local) << "</link>\n";
    urdf << "<joint name=\"part_joint\" type=\"fixed\"><parent link=\"base_link\"/>"
         << "<child link=\"part\"/><origin " << originString(scene.part.pose)
         << "/></joint>\n";
  }

  for (std::size_t i = 0; i < scene.obstacles.size(); ++i) {
    const Obstacle& obstacle = scene.obstacles[i];
    if (obstacle.geometry.empty()) {
      continue;
    }
    const std::string link_name = "obstacle_" + std::to_string(i);
    urdf << "<link name=\"" << link_name << "\">" << inertialString()
         << collisionForGeometry(obstacle.geometry, obstacle.geometry.local) << "</link>\n";
    urdf << "<joint name=\"" << link_name << "_joint\" type=\"fixed\"><parent link=\"base_link\"/>"
         << "<child link=\"" << link_name << "\"/><origin " << originString(obstacle.pose)
         << "/></joint>\n";
  }

  if (scene.has_ground) {
    const Isometry ground_pose =
        makeIsometry(Vec3(0.0, 0.0, scene.ground_z - 0.1), Mat3::Identity());
    urdf << "<link name=\"ground\">" << inertialString()
         << "<collision><origin " << originString(ground_pose)
         << "/><geometry><box size=\"20 20 0.2\"/></geometry></collision></link>\n";
    urdf << "<joint name=\"ground_joint\" type=\"fixed\"><parent link=\"base_link\"/>"
         << "<child link=\"ground\"/><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/></joint>\n";
  }

  urdf << "</robot>\n";
  return urdf.str();
}

std::string makeSrdf(const SceneModel& scene, const std::string& kinematics_yaml,
                     const std::string& contact_yaml) {
  const SerialChain& chain = scene.robot;
  std::ostringstream srdf;
  srdf << "<robot name=\"" << (chain.name.empty() ? "robot" : chain.name) << "\">\n";
  srdf << "<kinematics_plugin_config filename=\"" << kinematics_yaml << "\"/>\n";
  srdf << "<contact_managers_plugin_config filename=\"" << contact_yaml << "\"/>\n";
  srdf << "<group name=\"manipulator\"><chain base_link=\"base_link\" tip_link=\"tool0\"/></group>\n";
  std::vector<std::string> robot_links{"base_link"};
  for (std::size_t i = 0; i < chain.joints.size(); ++i) {
    robot_links.push_back("link_" + std::to_string(i));
  }
  robot_links.push_back("tool0");
  for (std::size_t i = 0; i < robot_links.size(); ++i) {
    for (std::size_t j = i + 1; j < robot_links.size(); ++j) {
      srdf << "<disable_collisions link1=\"" << robot_links[i] << "\" link2=\""
           << robot_links[j] << "\" reason=\"Adjacent\"/>\n";
    }
  }
  srdf << "</robot>\n";
  return srdf.str();
}

}  // namespace

std::string tesseractPluginDirectory() {
#ifdef RP_TESSERACT_BIN_DIR
  std::string path = RP_TESSERACT_BIN_DIR;
  std::replace(path.begin(), path.end(), '\\', '/');
  return path;
#else
  return ".";
#endif
}

bool writeTextFile(const std::filesystem::path& path, const std::string& content,
                   std::string* error) {
  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    if (error != nullptr) {
      *error = "failed to write Tesseract plugin config: " + path.string();
    }
    return false;
  }
  stream << content;
  if (!stream) {
    if (error != nullptr) {
      *error = "failed to write Tesseract plugin config: " + path.string();
    }
    return false;
  }
  return true;
}

std::string fileUri(const std::filesystem::path& path) {
  return std::filesystem::absolute(path).generic_string();
}

#ifdef RP_WITH_TESSERACT

struct TesseractPlanner::Impl {
  std::string error;
  bool valid = false;
  std::shared_ptr<tesseract::environment::Environment> environment;
  std::vector<tesseract::common::JointId> joint_ids;
  tesseract::common::ManipulatorInfo manipulator_info;
};

#else

struct TesseractPlanner::Impl {
  std::string error = "Tesseract support was not enabled at build time";
  bool valid = false;
};

#endif

bool isTesseractAvailable() {
#ifdef RP_WITH_TESSERACT
  return true;
#else
  return false;
#endif
}

TesseractPlanner::TesseractPlanner(const SceneModel& scene) : impl_(std::make_unique<Impl>()) {
#ifdef RP_WITH_TESSERACT
  if (!validateChain(scene.robot, &impl_->error)) {
    impl_->error = "invalid robot chain: " + impl_->error;
    return;
  }
  tesseract::common::getLogger()->set_level(spdlog::level::err);
  const std::string plugin_directory = tesseractPluginDirectory();
  const std::filesystem::path temp_directory = std::filesystem::temp_directory_path();
  const std::filesystem::path kinematics_yaml =
      temp_directory / "robotic_planner_tesseract_kinematics.yaml";
  const std::filesystem::path contact_yaml =
      temp_directory / "robotic_planner_tesseract_contact_managers.yaml";

  std::ostringstream kinematics_config;
  kinematics_config << "kinematic_plugins:\n"
                    << "  search_paths:\n"
                    << "    - " << plugin_directory << "\n"
                    << "  search_libraries:\n"
                    << "    - tesseract_kinematics_kdl_factories\n"
                    << "  fwd_kin_plugins:\n"
                    << "    manipulator:\n"
                    << "      default: KDLFwdKinChain\n"
                    << "      plugins:\n"
                    << "        KDLFwdKinChain:\n"
                    << "          class: KDLFwdKinChainFactory\n"
                    << "          config:\n"
                    << "            base_link: base_link\n"
                    << "            tip_link: tool0\n"
                    << "  inv_kin_plugins:\n"
                    << "    manipulator:\n"
                    << "      default: KDLInvKinChainLMA\n"
                    << "      plugins:\n"
                    << "        KDLInvKinChainLMA:\n"
                    << "          class: KDLInvKinChainLMAFactory\n"
                    << "          config:\n"
                    << "            base_link: base_link\n"
                    << "            tip_link: tool0\n";

  std::ostringstream contact_config;
  contact_config << "contact_manager_plugins:\n"
                 << "  search_paths:\n"
                 << "    - " << plugin_directory << "\n"
                 << "  search_libraries:\n"
                 << "    - tesseract_collision_bullet_factories\n"
                 << "    - tesseract_collision_fcl_factories\n"
                 << "  discrete_plugins:\n"
                 << "    default: BulletDiscreteBVHManager\n"
                 << "    plugins:\n"
                 << "      BulletDiscreteBVHManager:\n"
                 << "        class: BulletDiscreteBVHManagerFactory\n"
                 << "      FCLDiscreteBVHManager:\n"
                 << "        class: FCLDiscreteBVHManagerFactory\n"
                 << "  continuous_plugins:\n"
                 << "    default: BulletCastBVHManager\n"
                 << "    plugins:\n"
                 << "      BulletCastBVHManager:\n"
                 << "        class: BulletCastBVHManagerFactory\n";

  if (!writeTextFile(kinematics_yaml, kinematics_config.str(), &impl_->error) ||
      !writeTextFile(contact_yaml, contact_config.str(), &impl_->error)) {
    return;
  }

  const std::string urdf = makeUrdf(scene);
  const std::string srdf = makeSrdf(scene, fileUri(kinematics_yaml), fileUri(contact_yaml));
  auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
  impl_->environment = std::make_shared<tesseract::environment::Environment>();
  if (!impl_->environment->init(urdf, srdf, locator)) {
    impl_->error = "failed to initialize Tesseract environment";
    return;
  }
  for (const Joint& joint : scene.robot.joints) {
    impl_->joint_ids.emplace_back(joint.name);
  }
  impl_->manipulator_info = tesseract::common::ManipulatorInfo("manipulator", "base_link", "tool0");
  impl_->valid = true;
#else
  (void)scene;
#endif
}

TesseractPlanner::~TesseractPlanner() = default;
TesseractPlanner::TesseractPlanner(TesseractPlanner&&) noexcept = default;
TesseractPlanner& TesseractPlanner::operator=(TesseractPlanner&&) noexcept = default;

bool TesseractPlanner::valid() const { return impl_ != nullptr && impl_->valid; }

const std::string& TesseractPlanner::error() const {
  static const std::string empty;
  return impl_ == nullptr ? empty : impl_->error;
}

bool TesseractPlanner::planJointPath(const VecX& start, const VecX& goal, double time_limit,
                                     std::vector<VecX>* path, std::string* error) const {
  if (path == nullptr) {
    return false;
  }
  path->clear();
  if (!valid()) {
    if (error != nullptr) {
      *error = this->error();
    }
    return false;
  }
#ifdef RP_WITH_TESSERACT
  if (start.size() != static_cast<Eigen::Index>(impl_->joint_ids.size()) ||
      goal.size() != start.size()) {
    if (error != nullptr) {
      *error = "joint dimension does not match the Tesseract environment";
    }
    return false;
  }

  impl_->environment->setState(impl_->joint_ids, start);

  tesseract::command_language::CompositeInstruction program("FREESPACE",
                                                            impl_->manipulator_info);
  tesseract::command_language::StateWaypoint start_waypoint(impl_->joint_ids, start);
  tesseract::command_language::StateWaypoint goal_waypoint(impl_->joint_ids, goal);
  tesseract::command_language::MoveInstruction start_instruction(
      start_waypoint, tesseract::command_language::MoveInstructionType::FREESPACE, "FREESPACE");
  tesseract::command_language::MoveInstruction goal_instruction(
      goal_waypoint, tesseract::command_language::MoveInstructionType::FREESPACE, "FREESPACE");
  program.push_back(start_instruction);
  program.push_back(goal_instruction);

  auto profile = std::make_shared<tesseract::motion_planners::OMPLRealVectorMoveProfile>();
  profile->solver_config.planning_time = std::max(0.01, time_limit);
  profile->solver_config.optimize = false;
  profile->solver_config.max_solutions = 1;
  profile->solver_config.simplify = true;
  profile->solver_config.simplify_time = 0.05;
  profile->solver_config.planners = {
      std::make_shared<tesseract::motion_planners::RRTConnectConfigurator>()};
  auto profiles = std::make_shared<tesseract::common::ProfileDictionary>();
  profiles->addProfile("TesseractOMPL", "FREESPACE", profile);

  tesseract::motion_planners::PlannerRequest request;
  request.env = impl_->environment;
  request.profiles = profiles;
  request.instructions = std::move(program);

  tesseract::motion_planners::OMPLMotionPlanner planner("TesseractOMPL");
  const tesseract::motion_planners::PlannerResponse response = planner.solve(request);
  if (!response) {
    if (error != nullptr) {
      *error = response.message.empty() ? "Tesseract OMPL planning failed" : response.message;
    }
    return false;
  }

  for (const auto& instruction : response.results) {
    if (!instruction.isMoveInstruction()) {
      continue;
    }
    const auto& move = instruction.as<tesseract::command_language::MoveInstructionPoly>();
    const Eigen::VectorXd& q =
        tesseract::command_language::getJointPosition(move.getWaypoint());
    if (q.size() == start.size()) {
      path->push_back(q);
    }
  }

  if (path->size() < 2) {
    if (error != nullptr) {
      *error = "Tesseract returned an empty trajectory";
    }
    return false;
  }
  if ((path->front() - start).norm() > 1e-4 || (path->back() - goal).norm() > 1e-4) {
    if (error != nullptr) {
      *error = "Tesseract trajectory endpoints do not match the request";
    }
    return false;
  }
  return true;
#else
  (void)start;
  (void)goal;
  (void)time_limit;
  (void)error;
  return false;
#endif
}

}  // namespace rp
