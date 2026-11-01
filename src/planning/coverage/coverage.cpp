#include "rp/planning/coverage/coverage.h"

#include <algorithm>
#include <cmath>

namespace rp {

Vec3 TargetSurface::resolvedNormal() const {
  const Vec3 u = normalizedOr(u_axis, Vec3::UnitX());
  const Vec3 v = normalizedOr(v_axis, Vec3::UnitY());
  if (normal.norm() > kEpsilon) {
    return normal.normalized();
  }
  return normalizedOr(u.cross(v), Vec3::UnitZ());
}

TargetSurface makeSubSurface(const TargetSurface& surface, double center_u, double center_v,
                             double size_u, double size_v) {
  TargetSurface patch;
  patch.name = surface.name + "_patch";
  const Vec3 u_axis = normalizedOr(surface.u_axis, Vec3::UnitX());
  const Vec3 v_axis = normalizedOr(surface.v_axis, Vec3::UnitY());
  patch.u_axis = u_axis;
  patch.v_axis = v_axis;
  patch.normal = surface.resolvedNormal();
  patch.size_u = std::min(std::max(1e-4, size_u), surface.size_u);
  patch.size_v = std::min(std::max(1e-4, size_v), surface.size_v);
  const double half_u = patch.size_u * 0.5;
  const double half_v = patch.size_v * 0.5;
  const double clamped_u = std::min(std::max(center_u, half_u), surface.size_u - half_u);
  const double clamped_v = std::min(std::max(center_v, half_v), surface.size_v - half_v);
  patch.origin = surface.origin + u_axis * (clamped_u - half_u) + v_axis * (clamped_v - half_v);
  return patch;
}

std::vector<ScanStrip> CoverageGenerator::generateStrips(const TargetSurface& surface,
                                                          const ScannerParams& scanner,
                                                          bool sweep_along_u) {
  std::vector<ScanStrip> strips;
  if (surface.size_u <= 0.0 || surface.size_v <= 0.0 || scanner.fov_width <= 0.0 ||
      scanner.sample_spacing <= 0.0) {
    return strips;
  }

  const Vec3 u_axis = normalizedOr(surface.u_axis, Vec3::UnitX());
  const Vec3 v_axis = normalizedOr(surface.v_axis, Vec3::UnitY());
  const Vec3 normal = normalizedOr(surface.resolvedNormal(), normalizedOr(u_axis.cross(v_axis), Vec3::UnitZ()));

  const double sweep_size = sweep_along_u ? surface.size_u : surface.size_v;
  const double cross_size = sweep_along_u ? surface.size_v : surface.size_u;
  const Vec3 sweep_axis = sweep_along_u ? u_axis : v_axis;

  const double overlap = std::min(0.95, std::max(0.0, scanner.overlap));
  const double strip_step = std::max(1e-4, scanner.fov_width * (1.0 - overlap));
  const int strip_count = std::max(1, static_cast<int>(std::ceil(cross_size / strip_step)));
  const int samples_per_strip = std::max(2, static_cast<int>(std::round(sweep_size / scanner.sample_spacing)));

  Vec3 tangent = sweep_axis - normal * sweep_axis.dot(normal);
  if (tangent.norm() < kEpsilon) {
    const Vec3 alternative = sweep_along_u ? v_axis : u_axis;
    tangent = alternative - normal * alternative.dot(normal);
  }
  if (tangent.norm() < kEpsilon) {
    tangent = normalizedOr(normal.cross(Vec3::UnitZ()), Vec3::UnitX());
  }
  tangent.normalize();

  Vec3 tool_x = tangent;
  Vec3 tool_z = -normal;
  Vec3 tool_y = tool_z.cross(tool_x);
  if (tool_y.norm() < kEpsilon) {
    tool_x = normalizedOr(normal.cross(Vec3::UnitX()), Vec3::UnitY());
    tool_y = tool_z.cross(tool_x);
  }
  tool_y.normalize();
  tool_x = tool_y.cross(tool_z).normalized();

  Mat3 rotation;
  rotation.col(0) = tool_x;
  rotation.col(1) = tool_y;
  rotation.col(2) = tool_z;

  for (int strip_index = 0; strip_index < strip_count; ++strip_index) {
    const double cross_center = (static_cast<double>(strip_index) + 0.5) * cross_size /
                                static_cast<double>(strip_count);

    ScanStrip strip;
    strip.id = strip_index;
    strip.v = cross_center;

    for (int sample_index = 0; sample_index < samples_per_strip; ++sample_index) {
      const double sweep_coordinate =
          (static_cast<double>(sample_index) + 0.5) * sweep_size /
          static_cast<double>(samples_per_strip);

      const double u = sweep_along_u ? sweep_coordinate : cross_center;
      const double v = sweep_along_u ? cross_center : sweep_coordinate;

      ScanSample sample;
      sample.u = u;
      sample.v = v;
      sample.point = surface.origin + u_axis * u + v_axis * v;
      sample.normal = normal;
      sample.tangent = tangent;
      sample.tool_pose = makeIsometry(sample.point + normal * scanner.standoff, rotation);

      if (!strip.samples.empty()) {
        strip.length_m += (sample.point - strip.samples.back().point).norm();
      }
      strip.samples.push_back(sample);
    }
    strips.push_back(std::move(strip));
  }

  return strips;
}

CoverageGrid::CoverageGrid(const TargetSurface& surface, double cell_size)
    : surface_(surface), cell_size_(std::max(1e-4, cell_size)) {
  nu_ = std::max(1, static_cast<int>(std::ceil(surface.size_u / cell_size_)));
  nv_ = std::max(1, static_cast<int>(std::ceil(surface.size_v / cell_size_)));
  du_ = surface.size_u / static_cast<double>(nu_);
  dv_ = surface.size_v / static_cast<double>(nv_);
  total_cells_ = nu_ * nv_;
  cells_.assign(static_cast<std::size_t>(total_cells_), 0);
}

void CoverageGrid::addFootprint(double u, double v, double half_u, double half_v) {
  if (half_u <= 0.0 || half_v <= 0.0) {
    return;
  }
  const double u_min = std::max(0.0, u - half_u);
  const double u_max = std::min(surface_.size_u, u + half_u);
  const double v_min = std::max(0.0, v - half_v);
  const double v_max = std::min(surface_.size_v, v + half_v);
  if (u_min >= u_max || v_min >= v_max) {
    return;
  }

  const double epsilon = 1e-9 * std::max({1.0, surface_.size_u, surface_.size_v});
  const int i0 = std::max(0, static_cast<int>(std::floor(u_min / du_)));
  const int i1 = std::min(nu_ - 1, static_cast<int>(std::floor((u_max + epsilon) / du_)));
  const int j0 = std::max(0, static_cast<int>(std::floor(v_min / dv_)));
  const int j1 = std::min(nv_ - 1, static_cast<int>(std::floor((v_max + epsilon) / dv_)));

  for (int j = j0; j <= j1; ++j) {
    for (int i = i0; i <= i1; ++i) {
      const double center_u = (static_cast<double>(i) + 0.5) * du_;
      const double center_v = (static_cast<double>(j) + 0.5) * dv_;
      if (center_u < u_min - epsilon || center_u > u_max + epsilon ||
          center_v < v_min - epsilon || center_v > v_max + epsilon) {
        continue;
      }
      const std::size_t index = static_cast<std::size_t>(j) * static_cast<std::size_t>(nu_) +
                                static_cast<std::size_t>(i);
      if (cells_[index] == 0) {
        cells_[index] = 1;
        ++marked_cells_;
      }
    }
  }
}

double CoverageGrid::coveredArea() const {
  return static_cast<double>(marked_cells_) * du_ * dv_;
}

double CoverageGrid::ratio() const {
  const double area = surface_.size_u * surface_.size_v;
  if (area <= kEpsilon) {
    return 0.0;
  }
  return std::min(1.0, coveredArea() / area);
}

}  // namespace rp
