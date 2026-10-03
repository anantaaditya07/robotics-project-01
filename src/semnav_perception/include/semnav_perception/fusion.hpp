// Bearing-range fusion of a 2D detection with a 2D LiDAR scan (architecture 7.2).
// ROS-free, header-only, plain STL. The node (semantic_fusion_node) supplies
// camera intrinsics from /camera/camera_info, the optical->laser transform from
// TF at the image stamp, the closest scan, and the class radius; this module
// returns a position in the LASER frame. Transforming to map (step 6) is done by
// the node with TF.
//
// Algorithm (7.2; step 4 deviates per D-22):
//  1) Rays r = ((u - cx)/fx, 0, 1) in the camera optical frame (x right, y down,
//     z forward) for u_min, u_max and the bbox centre u_c = (u_min + u_max)/2.
//  2) Rotate rays into the laser frame, yaw = atan2(y, x).
//  3) Shrink the sector [yaw_lo, yaw_hi] symmetrically about its angular centre
//     to sector_fraction of its width; collect scan beams inside it.
//  4) Reject NaN/inf/out-of-range values. D-22 (accepted deviation from the PDF
//     "median"): sort the valid ranges, split them into clusters wherever two
//     consecutive sorted values differ by MORE than cluster_gap (a difference
//     exactly equal to cluster_gap stays in the same cluster), and take the
//     median of the NEAREST cluster with at least min_cluster_beams beams (even
//     count: mean of the two middle values). This ignores background seen
//     between thin object parts (e.g. the wall between a person's legs).
//     No valid value / no qualifying cluster -> failure, never a guess.
//  5) bearing = yaw of the centre ray; per D-10 the LiDAR hits the near surface,
//     so the point is pushed outward along the ray by the class radius:
//     p = (range + radius * push_out_fraction) * (cos bearing, sin bearing) (D-21).
//
// Angles are compared through normalised differences, so sectors straddling the
// 0 / 2*pi seam of the TurtleBot3 scan (angle 0 .. 2*pi, D-07) work, as do
// scans with a negative angle_increment.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace semnav_perception::fusion {

/// Numerical tolerance for "beam angle lies on the sector boundary" and for a
/// degenerate (vertical) rotated ray. Not a tunable: it only absorbs floating
/// point round-off.
constexpr double kAngleEpsilon = 1e-9;

/// pi (M_PI is not standard C++).
constexpr double kPi = 3.14159265358979323846;

/// Tunables (all ROS parameters in the node).
struct Params {
  double sector_fraction;  ///< central fraction of the bbox sector used, in (0, 1]
  double min_range;        ///< [m] lower range limit, combined with scan.range_min
  double max_range;        ///< [m] upper range limit, combined with scan.range_max
  /// Fraction of the class radius the LiDAR point is pushed out along the ray, in [0, 1].
  /// 1.0 = full radius (D-10 literal); smaller values compensate for the LiDAR hitting a part of
  /// the object (e.g. legs) that is already behind its near surface. 1.0 if not set.
  double push_out_fraction{1.0};
  /// [m] D-22: consecutive sorted valid ranges differing by more than this start a new cluster;
  /// must be > 0. Default 0.3 m: larger than the depth spread of one object's visible surface
  /// (a few cm to ~0.2 m), smaller than the typical object-to-background distance.
  double cluster_gap{0.3};
  /// D-22: a cluster needs at least this many beams to be used (nearer, smaller clusters are
  /// skipped); must be >= 1. Default 2: a single stray beam is never taken as the object, while
  /// a small/far object covering two beams still fuses.
  std::size_t min_cluster_beams{2};
};

/// Throws std::invalid_argument if `p` is invalid:
/// sector_fraction must be in (0, 1], min_range >= 0, min_range < max_range,
/// push_out_fraction in [0, 1], cluster_gap > 0 (finite), min_cluster_beams >= 1.
inline void validate(const Params& p) {
  if (!(p.sector_fraction > 0.0 && p.sector_fraction <= 1.0)) {
    throw std::invalid_argument("fusion: sector_fraction must be in (0, 1], got " +
                                std::to_string(p.sector_fraction));
  }
  if (!(p.min_range >= 0.0)) {
    throw std::invalid_argument("fusion: min_range must be >= 0, got " +
                                std::to_string(p.min_range));
  }
  if (!(p.min_range < p.max_range)) {
    throw std::invalid_argument("fusion: min_range must be < max_range, got " +
                                std::to_string(p.min_range) + " >= " + std::to_string(p.max_range));
  }
  if (!(p.push_out_fraction >= 0.0 && p.push_out_fraction <= 1.0)) {
    throw std::invalid_argument("fusion: push_out_fraction must be in [0, 1], got " +
                                std::to_string(p.push_out_fraction));
  }
  if (!(p.cluster_gap > 0.0 && std::isfinite(p.cluster_gap))) {
    throw std::invalid_argument("fusion: cluster_gap must be finite and > 0, got " +
                                std::to_string(p.cluster_gap));
  }
  if (p.min_cluster_beams < 1) {
    throw std::invalid_argument("fusion: min_cluster_beams must be >= 1, got " +
                                std::to_string(p.min_cluster_beams));
  }
}

/// Horizontal pinhole intrinsics, from sensor_msgs/CameraInfo (fx = K[0], cx = K[2]).
struct Intrinsics {
  double fx;  ///< [px] focal length, must be > 0
  double cx;  ///< [px] principal point
};

/// Transform optical -> laser: p_laser = R * p_optical + t.
/// `rotation` is a 3x3 row-major matrix (R[3*row + col]).
///
/// Only the rotation is used to compute bearings (as 7.2 specifies). The
/// camera-laser translation (a few cm on the Waffle) is deliberately ignored:
/// the bearing error it causes is roughly |t_lateral| / range (e.g. ~5 cm at
/// 2 m -> ~0.025 rad), small compared to the bbox sector width. Callers still
/// pass the full transform so a future version can correct for it.
struct Transform {
  std::array<double, 9> rotation;
  std::array<double, 3> translation;
};

/// sensor_msgs/LaserScan geometry (ranges are passed separately).
/// Beam i has angle angle_min + i * angle_increment.
struct ScanGeometry {
  double angle_min;        ///< [rad]
  double angle_increment;  ///< [rad], non-zero (sign may be negative)
  double range_min;        ///< [m]
  double range_max;        ///< [m]
};

enum class Status {
  Ok,
  BadInput,       ///< non-finite / inconsistent bbox, intrinsics, rotation, scan geometry, radius
  EmptySector,    ///< no scan beam falls inside the (shrunk) sector
  NoValidRanges,  ///< beams in the sector, but all rejected as invalid / out of range
  NoCluster,      ///< valid ranges, but no cluster has min_cluster_beams beams (D-22)
};

inline const char* toString(Status s) {
  switch (s) {
    case Status::Ok:
      return "Ok";
    case Status::BadInput:
      return "BadInput";
    case Status::EmptySector:
      return "EmptySector";
    case Status::NoValidRanges:
      return "NoValidRanges";
    case Status::NoCluster:
      return "NoCluster";
  }
  return "Unknown";
}

/// Fusion output, all in the laser frame. On failure, fields computed before the
/// failing step are still filled (useful for logging); `x`, `y`, `range` are 0.
struct Result {
  Status status{Status::BadInput};
  double bearing{0.0};  ///< [rad] yaw of the bbox centre ray, in (-pi, pi]
  double range{0.0};    ///< [m] LiDAR range used: nearest-cluster median (D-22), before push-out
  double x{0.0};        ///< [m] position, pushed out by the class radius
  double y{0.0};        ///< [m]
  std::size_t beams_in_sector{0};
  std::size_t valid_beams{0};
  std::size_t cluster_beams{0};  ///< beams in the chosen (nearest qualifying) cluster, D-22
  double sector_lo{0.0};         ///< [rad] shrunk sector start, in (-pi, pi]
  double sector_hi{0.0};         ///< [rad] shrunk sector end, in (-pi, pi]; < sector_lo if it wraps
  [[nodiscard]] bool ok() const { return status == Status::Ok; }
};

/// Wraps an angle to (-pi, pi].
inline double normalizeAngle(double a) {
  const double two_pi = 2.0 * kPi;
  a = std::fmod(a, two_pi);  // (-2pi, 2pi)
  if (a > kPi) {
    a -= two_pi;
  } else if (a <= -kPi) {
    a += two_pi;
  }
  return a;
}

/// Yaw in the laser frame of the optical ray through image column `u`.
/// Returns NaN if the rotated ray is (numerically) vertical in the laser frame.
inline double rayYaw(double u, const Intrinsics& k, const std::array<double, 9>& r) {
  // Optical ray: ((u - cx) / fx, 0, 1). Rotated: R * ray (y component is 0).
  const double ox = (u - k.cx) / k.fx;
  const double oz = 1.0;
  const double lx = r[0] * ox + r[2] * oz;
  const double ly = r[3] * ox + r[5] * oz;
  if (std::hypot(lx, ly) < kAngleEpsilon) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::atan2(ly, lx);
}

/// Angular sector given by its centre and half-width (centre in (-pi, pi]).
struct Sector {
  double centre;
  double half_width;
  [[nodiscard]] double lo() const { return normalizeAngle(centre - half_width); }
  [[nodiscard]] double hi() const { return normalizeAngle(centre + half_width); }
  [[nodiscard]] bool contains(double angle) const {
    return std::abs(normalizeAngle(angle - centre)) <= half_width + kAngleEpsilon;
  }
};

/// Sector spanned by the two edge yaws (the shorter arc between them; a camera
/// FOV is < pi), shrunk symmetrically about its centre to `fraction` of its width.
inline Sector shrinkSector(double yaw_a, double yaw_b, double fraction) {
  const double diff = normalizeAngle(yaw_b - yaw_a);  // signed arc a -> b
  const double centre = normalizeAngle(yaw_a + 0.5 * diff);
  return Sector{centre, 0.5 * std::abs(diff) * fraction};
}

/// Median of `v` (reordered in place). Even count: mean of the two middle
/// values. Precondition: !v.empty().
inline double median(std::vector<double>& v) {
  const std::size_t n = v.size();
  const std::size_t mid = n / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
  const double upper = v[mid];
  if (n % 2 == 1) {
    return upper;
  }
  const double lower = *std::max_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid));
  return 0.5 * (lower + upper);
}

/// D-22: median of the nearest cluster of `ranges` with at least `min_beams` values.
/// Sorted values are split into clusters where consecutive values differ by MORE than `gap`
/// (difference == gap stays in the same cluster). Clusters are scanned nearest first; the first
/// one with >= min_beams values is used. Returns std::nullopt if `ranges` is empty or no cluster
/// qualifies (never a guess). If `cluster_size` is non-null it receives the size of the chosen
/// cluster (0 on failure). Preconditions: values finite, gap > 0.
inline std::optional<double> nearestClusterMedian(std::vector<double> ranges, double gap,
                                                  std::size_t min_beams,
                                                  std::size_t* cluster_size = nullptr) {
  if (cluster_size != nullptr) {
    *cluster_size = 0;
  }
  std::sort(ranges.begin(), ranges.end());
  std::size_t begin = 0;
  while (begin < ranges.size()) {
    std::size_t end = begin + 1;  // one past the last element of this cluster
    while (end < ranges.size() && ranges[end] - ranges[end - 1] <= gap) {
      ++end;
    }
    if (end - begin >= min_beams) {
      std::vector<double> cluster(ranges.begin() + static_cast<std::ptrdiff_t>(begin),
                                  ranges.begin() + static_cast<std::ptrdiff_t>(end));
      if (cluster_size != nullptr) {
        *cluster_size = cluster.size();
      }
      return median(cluster);
    }
    begin = end;
  }
  return std::nullopt;
}

/// Full 7.2 fusion (steps 1-5). Throws std::invalid_argument on invalid `params`
/// (see validate()); every other problem is reported through Result::status.
///
/// @param u_min, u_max  bbox horizontal extent [px] in the original image, u_min < u_max
/// @param k             intrinsics from CameraInfo
/// @param tf            optical -> laser transform at the image stamp
/// @param scan          LaserScan geometry
/// @param ranges        LaserScan ranges (float, as in the message)
/// @param radius        class radius [m] (>= 0) for the outward push (D-10)
inline Result fuse(double u_min, double u_max, const Intrinsics& k, const Transform& tf,
                   const ScanGeometry& scan, const std::vector<float>& ranges, double radius,
                   const Params& params) {
  validate(params);
  Result res;

  const bool rotation_finite = std::all_of(tf.rotation.begin(), tf.rotation.end(),
                                           [](double v) { return std::isfinite(v); });
  if (!std::isfinite(u_min) || !std::isfinite(u_max) || !(u_min < u_max) || !std::isfinite(k.fx) ||
      !(k.fx > 0.0) || !std::isfinite(k.cx) || !rotation_finite || !std::isfinite(scan.angle_min) ||
      !std::isfinite(scan.angle_increment) || scan.angle_increment == 0.0 ||
      !std::isfinite(radius) || !(radius >= 0.0)) {
    res.status = Status::BadInput;
    return res;
  }

  // Steps 1-2: edge and centre rays -> laser-frame yaw.
  const double yaw_a = rayYaw(u_min, k, tf.rotation);
  const double yaw_b = rayYaw(u_max, k, tf.rotation);
  const double bearing = rayYaw(0.5 * (u_min + u_max), k, tf.rotation);
  if (std::isnan(yaw_a) || std::isnan(yaw_b) || std::isnan(bearing)) {
    res.status = Status::BadInput;
    return res;
  }
  res.bearing = bearing;

  // Step 3: central sector_fraction of the sector.
  const Sector sector = shrinkSector(yaw_a, yaw_b, params.sector_fraction);
  res.sector_lo = sector.lo();
  res.sector_hi = sector.hi();

  // Step 4: collect, reject invalid, nearest-cluster median (D-22).
  const double lo_lim = std::max(scan.range_min, params.min_range);
  const double hi_lim = std::min(scan.range_max, params.max_range);
  std::vector<double> valid;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (!sector.contains(angle)) {
      continue;
    }
    ++res.beams_in_sector;
    const double r = static_cast<double>(ranges[i]);
    if (std::isfinite(r) && r >= lo_lim && r <= hi_lim) {
      valid.push_back(r);
    }
  }
  res.valid_beams = valid.size();
  if (res.beams_in_sector == 0) {
    res.status = Status::EmptySector;
    return res;
  }
  if (valid.empty()) {
    res.status = Status::NoValidRanges;
    return res;
  }
  const std::optional<double> range = nearestClusterMedian(
      std::move(valid), params.cluster_gap, params.min_cluster_beams, &res.cluster_beams);
  if (!range) {
    res.status = Status::NoCluster;
    return res;
  }
  res.range = *range;

  // Step 5: push outward along the centre ray by a fraction of the class radius (D-10, D-21).
  const double d = res.range + radius * params.push_out_fraction;
  res.x = d * std::cos(bearing);
  res.y = d * std::sin(bearing);
  res.status = Status::Ok;
  return res;
}

/// D-25 negative evidence: is a point (camera OPTICAL frame: x right, y down, z forward) where
/// the camera and LiDAR could see it? True if z > 0, its image column u = fx * x / z + cx lies in
/// [margin_px, width - margin_px], and its horizontal range hypot(x, z) is in [min_range,
/// max_range]. Vertical extent is not checked (the 2D LiDAR defines the relevant height).
inline bool pointInView(double x, double z, const Intrinsics& k, double width_px, double margin_px,
                        double min_range, double max_range) {
  if (!(z > 0.0) || !(k.fx > 0.0) || !std::isfinite(x) || !std::isfinite(z)) {
    return false;
  }
  const double u = k.fx * x / z + k.cx;
  const double r = std::hypot(x, z);
  return u >= margin_px && u <= width_px - margin_px && r >= min_range && r <= max_range;
}

}  // namespace semnav_perception::fusion
