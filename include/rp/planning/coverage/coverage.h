#pragma once

#include "rp/core/types.h"

#include <string>
#include <vector>

namespace rp {

struct TargetSurface {
  std::string name = "surface";
  Vec3 origin = Vec3::Zero();
  Vec3 u_axis = Vec3::UnitX();
  Vec3 v_axis = Vec3::UnitY();
  double size_u = 1.0;
  double size_v = 1.0;
  Vec3 normal = Vec3::Zero();

  Vec3 resolvedNormal() const;
};

struct ScannerParams {
  double standoff = 0.15;
  double min_standoff = 0.03;
  double max_standoff = 0.50;
  double fov_width = 0.10;
  double overlap = 0.15;
  double sample_spacing = 0.02;
  double scan_speed = 0.15;
  double max_incidence_deg = 60.0;
};

struct ScanSample {
  Vec3 point = Vec3::Zero();
  Vec3 normal = Vec3::UnitZ();
  Vec3 tangent = Vec3::UnitX();
  Isometry tool_pose = Isometry::Identity();
  double u = 0.0;
  double v = 0.0;
};

struct ScanStrip {
  int id = 0;
  double v = 0.0;
  double length_m = 0.0;
  std::vector<ScanSample> samples;
};

TargetSurface makeSubSurface(const TargetSurface& surface, double center_u, double center_v,
                             double size_u, double size_v);

class CoverageGenerator {
 public:
  static std::vector<ScanStrip> generateStrips(const TargetSurface& surface,
                                                const ScannerParams& scanner,
                                                bool sweep_along_u = true);
};

class CoverageGrid {
 public:
  CoverageGrid(const TargetSurface& surface, double cell_size);

  void addFootprint(double u, double v, double half_u, double half_v);
  double coveredArea() const;
  double ratio() const;
  int markedCells() const { return marked_cells_; }
  int totalCells() const { return total_cells_; }
  double cellSize() const { return cell_size_; }

 private:
  TargetSurface surface_;
  double cell_size_ = 0.01;
  int nu_ = 0;
  int nv_ = 0;
  double du_ = 0.0;
  double dv_ = 0.0;
  int marked_cells_ = 0;
  int total_cells_ = 0;
  std::vector<unsigned char> cells_;
};

}  // namespace rp
