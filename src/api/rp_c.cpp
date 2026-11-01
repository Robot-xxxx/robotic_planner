#include "rp/api/rp_c.h"

#include "rp/api/scene_io.h"
#include "rp/planning/planner.h"

#include <cstring>
#include <exception>
#include <string>

#ifndef RP_VERSION_STRING
#define RP_VERSION_STRING "0.0.0"
#endif

namespace {

char* duplicateString(const std::string& value) {
  char* text = new char[value.size() + 1];
  std::memcpy(text, value.c_str(), value.size() + 1);
  return text;
}

void setError(char** error_message, const std::string& message) {
  if (error_message != nullptr) {
    *error_message = duplicateString(message);
  }
}

}  // namespace

extern "C" {

const char* rp_version(void) { return RP_VERSION_STRING; }

int rp_plan_file(const char* config_path, const char* output_path, char** error_message) {
  if (error_message != nullptr) {
    *error_message = nullptr;
  }
  if (config_path == nullptr) {
    setError(error_message, "config_path is null");
    return 1;
  }
  try {
    const rp::SceneConfig config = rp::loadSceneConfig(config_path);
    const rp::PlanResult result = rp::planScene(config);
    if (output_path != nullptr && !rp::savePlanJson(output_path, config, result)) {
      setError(error_message, "failed to write plan output");
      return 1;
    }
    if (!result.success) {
      setError(error_message, result.message);
      return 1;
    }
    return 0;
  } catch (const std::exception& exception) {
    setError(error_message, exception.what());
    return 1;
  } catch (...) {
    setError(error_message, "unknown planning failure");
    return 1;
  }
}

int rp_plan_json(const char* config_json, char** plan_json, char** error_message) {
  if (error_message != nullptr) {
    *error_message = nullptr;
  }
  if (plan_json != nullptr) {
    *plan_json = nullptr;
  }
  if (config_json == nullptr || plan_json == nullptr) {
    setError(error_message, "config_json and plan_json are required");
    return 1;
  }
  try {
    const rp::SceneConfig config = rp::parseSceneConfig(config_json);
    const rp::PlanResult result = rp::planScene(config);
    *plan_json = duplicateString(rp::planResultToJson(config, result));
    if (!result.success) {
      setError(error_message, result.message);
      return 1;
    }
    return 0;
  } catch (const std::exception& exception) {
    setError(error_message, exception.what());
    return 1;
  } catch (...) {
    setError(error_message, "unknown planning failure");
    return 1;
  }
}

void rp_free_string(char* text) { delete[] text; }

}  // extern "C"
