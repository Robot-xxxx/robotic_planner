#include "rp/api/scene_io.h"
#include "rp/planning/coverage/coverage.h"

#include <cmath>
#include <iostream>

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
  const rp::SceneConfig config = rp::loadSceneConfig(RP_DEFAULT_CONFIG);

  const std::vector<rp::ScanStrip> strips =
      rp::CoverageGenerator::generateStrips(config.surface, config.scanner, true);
  check(strips.size() == 9, "expected nine strips for the demo surface");
  check(!strips.empty() && strips.front().samples.size() == 30,
        "expected thirty samples per strip");

  int total_samples = 0;
  for (const rp::ScanStrip& strip : strips) {
    total_samples += static_cast<int>(strip.samples.size());
    check(strip.length_m > 0.0, "strip has positive length");
    for (const rp::ScanSample& sample : strip.samples) {
      const rp::Vec3 offset = sample.tool_pose.translation() - sample.point;
      check(std::abs(offset.norm() - config.scanner.standoff) < 1e-9,
            "sample keeps scanner standoff");
      check(std::abs(offset.normalized().dot(sample.normal) - 1.0) < 1e-9,
            "tool position is on the outward normal");
      check(std::abs(sample.tool_pose.linear().col(2).dot(sample.normal) + 1.0) < 1e-9,
            "tool z axis points into the surface");
    }
  }
  check(total_samples == 270, "expected 270 total samples");

  rp::CoverageGrid grid(config.surface, 0.01);
  const double half_u = 0.5 * config.surface.size_u / 30.0;
  const double half_v = 0.5 * config.scanner.fov_width;
  for (const rp::ScanStrip& strip : strips) {
    for (const rp::ScanSample& sample : strip.samples) {
      grid.addFootprint(sample.u, sample.v, half_u, half_v);
    }
  }
  check(std::abs(grid.ratio() - 1.0) < 1e-9, "all footprints cover the target completely");

  rp::CoverageGrid partial(config.surface, 0.01);
  partial.addFootprint(0.3, 0.35, 0.01, 0.05);
  check(partial.ratio() > 0.0 && partial.ratio() < 0.1, "single footprint covers a small area");

  const rp::TargetSurface tiny =
      rp::makeSubSurface(config.surface, 0.30, 0.35, 0.05, 0.05);
  check(std::abs(tiny.size_u - 0.05) < 1e-12 && std::abs(tiny.size_v - 0.05) < 1e-12,
        "sub-surface keeps requested size");
  rp::ScannerParams fine_scanner = config.scanner;
  fine_scanner.fov_width = 0.02;
  fine_scanner.sample_spacing = 0.0025;
  const std::vector<rp::ScanStrip> tiny_strips =
      rp::CoverageGenerator::generateStrips(tiny, fine_scanner, true);
  rp::CoverageGrid tiny_grid(tiny, 0.001);
  const double tiny_half_along = 0.5 * tiny.size_u / 20.0;
  const double tiny_half_cross = 0.5 * fine_scanner.fov_width;
  for (const rp::ScanStrip& strip : tiny_strips) {
    for (const rp::ScanSample& sample : strip.samples) {
      tiny_grid.addFootprint(sample.u, sample.v, tiny_half_along, tiny_half_cross);
    }
  }
  check(std::abs(tiny_grid.ratio() - 1.0) < 1e-12,
        "coverage grid is exact for fine 5 cm patches");

  const std::vector<rp::ScanStrip> vertical =
      rp::CoverageGenerator::generateStrips(config.surface, config.scanner, false);
  check(!vertical.empty(), "vertical sweep generates strips");
  check(vertical.front().samples.size() == 35, "vertical sweep sample count");
  check(std::abs(vertical.front().samples.front().tool_pose.linear().col(0).dot(
                     config.surface.v_axis.normalized()) -
                 1.0) <
            1e-9,
        "vertical sweep tangent follows the v axis");

  if (failures != 0) {
    std::cerr << failures << " coverage checks failed\n";
    return 1;
  }
  std::cout << "test_coverage passed\n";
  return 0;
}
