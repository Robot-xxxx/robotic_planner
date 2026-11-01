#include "rp/api/rp_c.h"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    ++failures;
  }
}

}  // namespace

int main() {
  check(std::string(rp_version()).size() > 0, "version string is available");

  std::ifstream stream(RP_DEFAULT_CONFIG, std::ios::binary);
  check(static_cast<bool>(stream), "scene config can be opened");
  if (!stream) {
    return 1;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  const std::string config_json = buffer.str();

  char* plan_json = nullptr;
  char* error = nullptr;
  const int json_status = rp_plan_json(config_json.c_str(), &plan_json, &error);
  check(json_status == 0, "C ABI JSON planning succeeds");
  if (error != nullptr) {
    std::cerr << "C ABI error: " << error << "\n";
    rp_free_string(error);
    error = nullptr;
  }
  check(plan_json != nullptr, "C ABI returns a plan JSON string");
  if (plan_json != nullptr) {
    const auto parsed = nlohmann::json::parse(plan_json);
    check(parsed.value("success", false), "C ABI plan reports success");
    check(parsed.contains("metrics"), "C ABI plan contains metrics");
    rp_free_string(plan_json);
    plan_json = nullptr;
  }

  const std::filesystem::path output =
      std::filesystem::temp_directory_path() / "rp_capi_test_plan.json";
  std::filesystem::remove(output);
  const int file_status = rp_plan_file(RP_DEFAULT_CONFIG, output.string().c_str(), &error);
  check(file_status == 0, "C ABI file planning succeeds");
  if (error != nullptr) {
    std::cerr << "C ABI file error: " << error << "\n";
    rp_free_string(error);
    error = nullptr;
  }
  check(std::filesystem::exists(output), "C ABI writes the output plan file");
  std::filesystem::remove(output);

  std::ifstream pose_stream(std::string(RP_SOURCE_DIR) + "/scenes/pose_sequence.json",
                            std::ios::binary);
  check(static_cast<bool>(pose_stream), "pose sequence config can be opened");
  if (pose_stream) {
    std::ostringstream pose_buffer;
    pose_buffer << pose_stream.rdbuf();
    char* pose_plan = nullptr;
    const int pose_status =
        rp_plan_json(pose_buffer.str().c_str(), &pose_plan, &error);
    check(pose_status == 0, "C ABI pose sequence planning succeeds");
    if (error != nullptr) {
      std::cerr << "C ABI pose error: " << error << "\n";
      rp_free_string(error);
      error = nullptr;
    }
    if (pose_plan != nullptr) {
      const auto parsed = nlohmann::json::parse(pose_plan);
      check(parsed.value("task", "") == "pose_sequence", "C ABI pose task type");
      check(parsed["metrics"].value("poses_reachable", 0) == 10,
            "C ABI pose sequence reachability");
      check(parsed.contains("pose_visits") && parsed["pose_visits"].size() == 10,
            "C ABI pose visits are serialized");
      rp_free_string(pose_plan);
    }
  }

  if (failures != 0) {
    std::cerr << failures << " C ABI checks failed\n";
    return 1;
  }
  std::cout << "test_capi passed\n";
  return 0;
}
