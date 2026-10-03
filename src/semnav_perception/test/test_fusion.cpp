// Unit tests for fusion.hpp (architecture 7.2). All expected values are hand-computed;
// the arithmetic is given in the comments.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "semnav_perception/fusion.hpp"

namespace fu = semnav_perception::fusion;

namespace {

constexpr double kPi = fu::kPi;
constexpr double kTol = 1e-9;
constexpr double kRangeTol = 1e-6;  // ranges are stored as float in LaserScan

// Simulated camera (640x480 sim camera, from /camera/camera_info).
constexpr double kFx = 565.6;
constexpr double kCx = 320.5;

// TB3 scan (D-07): 360 beams, angle 0 .. 2pi, range 0.12 .. 3.5 m.
constexpr std::size_t kBeams = 360;
constexpr double kScanRangeMin = 0.12;
constexpr double kScanRangeMax = 3.5;
constexpr double kDeg = kPi / 180.0;
constexpr float kWall = 3.0F;
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

using Mat3 = std::array<double, 9>;

// Forward-looking camera: x_laser = z_opt, y_laser = -x_opt, z_laser = -y_opt.
// Row-major R with p_laser = R * p_opt.
constexpr Mat3 kOpticalToLaser = {0, 0, 1, -1, 0, 0, 0, -1, 0};

Mat3 matMul(const Mat3& a, const Mat3& b) {
  Mat3 c{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        c[3 * i + j] += a[3 * i + k] * b[3 * k + j];
      }
    }
  }
  return c;
}

Mat3 rotZ(double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c, -s, 0, s, c, 0, 0, 0, 1};
}

fu::Transform forwardCamera() {
  // Translation is ignored for bearings; give a realistic Waffle offset anyway.
  return fu::Transform{kOpticalToLaser, {0.05, -0.05, 0.0}};
}

fu::Intrinsics simCamera() { return fu::Intrinsics{kFx, kCx}; }

fu::ScanGeometry tb3Scan() {
  return fu::ScanGeometry{0.0, 2.0 * kPi / static_cast<double>(kBeams), kScanRangeMin,
                          kScanRangeMax};
}

fu::Params defaultParams() { return fu::Params{0.6, 0.0, 10.0}; }

// Index of the TB3 beam at integer degree `deg` (may be negative: wraps to 360 + deg).
std::size_t beamAt(int deg) {
  const int n = static_cast<int>(kBeams);
  return static_cast<std::size_t>(((deg % n) + n) % n);
}

// Image column whose forward-camera ray has laser yaw `yaw`:
// yaw = -atan((u - cx) / fx)  =>  u = cx - fx * tan(yaw).
double uForYaw(double yaw) { return kCx - kFx * std::tan(yaw); }

}  // namespace

// ---------------------------------------------------------------- bearings

TEST(FusionBearing, CentredBboxHasZeroBearing) {
  // u_c = (270.5 + 370.5) / 2 = 320.5 = cx -> optical ray (0, 0, 1) -> laser (1, 0, 0) -> yaw 0.
  const std::vector<float> ranges(kBeams, kWall);
  const auto r = fu::fuse(kCx - 50.0, kCx + 50.0, simCamera(), forwardCamera(), tb3Scan(), ranges,
                          0.0, defaultParams());
  ASSERT_TRUE(r.ok()) << fu::toString(r.status);
  EXPECT_NEAR(r.bearing, 0.0, kTol);
}

TEST(FusionBearing, RayYawMatchesPinholeFormula) {
  // Optical ray ((u-cx)/fx, 0, 1) -> laser (1, -(u-cx)/fx, 0) -> yaw = -atan((u-cx)/fx).
  // e.g. u = 500: (500-320.5)/565.6 = 0.317362 -> yaw = -atan(0.317362) = -0.307308 rad.
  for (const double u : {0.0, 100.0, 320.5, 500.0, 639.0}) {
    const double expected = -std::atan((u - kCx) / kFx);
    EXPECT_NEAR(fu::rayYaw(u, simCamera(), kOpticalToLaser), expected, kTol) << "u=" << u;
  }
  EXPECT_NEAR(fu::rayYaw(500.0, simCamera(), kOpticalToLaser), -0.307308, 1e-6);
}

TEST(FusionBearing, BboxRightOfCentreHasNegativeYaw) {
  // Right in the image = -y in the laser frame. Bbox [420.5, 520.5], u_c = 470.5,
  // yaw = -atan(150 / 565.6) = -0.259125 rad.
  const std::vector<float> ranges(kBeams, kWall);
  const auto r =
      fu::fuse(420.5, 520.5, simCamera(), forwardCamera(), tb3Scan(), ranges, 0.0, defaultParams());
  ASSERT_TRUE(r.ok());
  EXPECT_LT(r.bearing, 0.0);
  EXPECT_NEAR(r.bearing, -std::atan(150.0 / kFx), kTol);
  EXPECT_LT(r.y, 0.0);
}

TEST(FusionBearing, YawedCameraShiftsBearingBy30Deg) {
  // Camera yawed +30 deg about the laser z axis: R = Rz(30deg) * R0, so every laser-frame
  // ray is rotated by +30 deg and yaw(u) = -atan((u-cx)/fx) + pi/6.
  const double yaw_offset = 30.0 * kDeg;
  const Mat3 rot = matMul(rotZ(yaw_offset), kOpticalToLaser);
  for (const double u : {0.0, 200.0, 320.5, 450.0, 639.0}) {
    EXPECT_NEAR(fu::rayYaw(u, simCamera(), rot), -std::atan((u - kCx) / kFx) + yaw_offset, kTol)
        << "u=" << u;
  }
  // Full fusion: centred bbox -> bearing pi/6. Put an object at 2.0 m on beams 25..35 deg.
  // Bbox [270.5, 370.5] spans yaw 30 -/+ atan(50/565.6) = 30 -/+ 5.05 deg; shrunk x0.6 ->
  // 30 +/- 3.03 deg -> beams 27..33 deg (7), all 2.0 -> median 2.0.
  // Position = (2.0 + 0.25) * (cos 30, sin 30) = (1.948557, 1.125).
  std::vector<float> ranges(kBeams, kWall);
  for (int d = 25; d <= 35; ++d) {
    ranges[beamAt(d)] = 2.0F;
  }
  const auto r = fu::fuse(kCx - 50.0, kCx + 50.0, simCamera(), fu::Transform{rot, {0, 0, 0}},
                          tb3Scan(), ranges, 0.25, defaultParams());
  ASSERT_TRUE(r.ok());
  EXPECT_NEAR(r.bearing, kPi / 6.0, kTol);
  EXPECT_EQ(r.beams_in_sector, 7U);
  EXPECT_NEAR(r.range, 2.0, kRangeTol);
  EXPECT_NEAR(r.x, 2.25 * std::cos(kPi / 6.0), kRangeTol);
  EXPECT_NEAR(r.y, 1.125, kRangeTol);
}

// ---------------------------------------------------------------- sector

TEST(FusionSector, ShrinkSectorDirect) {
  // [-0.2, 0.2]: centre 0, half-width 0.2 * 0.6 = 0.12 -> [-0.12, 0.12]. Order of edges is
  // irrelevant (forward camera gives u_min -> +yaw, u_max -> -yaw).
  for (const auto& s : {fu::shrinkSector(-0.2, 0.2, 0.6), fu::shrinkSector(0.2, -0.2, 0.6)}) {
    EXPECT_NEAR(s.centre, 0.0, kTol);
    EXPECT_NEAR(s.lo(), -0.12, kTol);
    EXPECT_NEAR(s.hi(), 0.12, kTol);
  }
  // Off-centre: [0.5, 0.9] -> centre 0.7, half 0.2 * 0.6 = 0.12 -> [0.58, 0.82].
  const auto s = fu::shrinkSector(0.5, 0.9, 0.6);
  EXPECT_NEAR(s.lo(), 0.58, kTol);
  EXPECT_NEAR(s.hi(), 0.82, kTol);
  // Across the +-pi seam: [pi-0.1, -pi+0.1] is a 0.2 rad arc centred on pi.
  const auto w = fu::shrinkSector(kPi - 0.1, -kPi + 0.1, 0.5);
  EXPECT_NEAR(std::abs(w.centre), kPi, kTol);
  EXPECT_NEAR(w.half_width, 0.05, kTol);
  EXPECT_TRUE(w.contains(kPi));
  EXPECT_TRUE(w.contains(-kPi + 0.04));
  EXPECT_FALSE(w.contains(-kPi + 0.06));
}

TEST(FusionSector, BboxSpanningPlusMinus0p2ShrinksTo0p12) {
  // u for yaw +0.2: 320.5 - 565.6 * tan(0.2) = 205.848; for yaw -0.2: 435.152.
  const std::vector<float> ranges(kBeams, kWall);
  const auto r = fu::fuse(uForYaw(0.2), uForYaw(-0.2), simCamera(), forwardCamera(), tb3Scan(),
                          ranges, 0.0, defaultParams());
  ASSERT_TRUE(r.ok());
  EXPECT_NEAR(r.sector_lo, -0.12, kTol);
  EXPECT_NEAR(r.sector_hi, 0.12, kTol);
  // Beams at k deg with |k| deg <= 0.12 rad = 6.875 deg -> k = -6..6 -> 13 beams.
  EXPECT_EQ(r.beams_in_sector, 13U);
}

// ---------------------------------------------------------------- median / rejection

TEST(FusionMedian, ObjectInFrontOfWallAndEdgeLeakage) {
  // Wall at 3.0 m everywhere; object at 1.5 m on beams -5..5 deg (11 beams). The bbox spans
  // yaw +-0.2 rad (+-11.46 deg), so wall pixels leak in at the edges (beams +-6..+-11).
  std::vector<float> ranges(kBeams, kWall);
  for (int d = -5; d <= 5; ++d) {
    ranges[beamAt(d)] = 1.5F;
  }
  const double u_min = uForYaw(0.2);
  const double u_max = uForYaw(-0.2);

  // sector_fraction 0.6 -> +-6.875 deg -> beams -6..6: 11 x 1.5 + 2 x 3.0 -> median 1.5.
  const auto r =
      fu::fuse(u_min, u_max, simCamera(), forwardCamera(), tb3Scan(), ranges, 0.0, defaultParams());
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.beams_in_sector, 13U);
  EXPECT_EQ(r.valid_beams, 13U);
  EXPECT_NEAR(r.range, 1.5, kRangeTol);

  EXPECT_EQ(r.cluster_beams, 11U);  // {1.5 x 11}; the 3.0 pair is 1.5 m away (> gap 0.3)

  // Without the shrink (fraction 1.0): beams -11..11 = 23: 11 x 1.5 + 12 x 3.0.
  // Expectation CHANGED by D-22: the plain median (PDF 7.2 step 4) was the 12th sorted value =
  // 3.0 (the wall); the nearest-cluster median now ignores the 12 wall beams (gap 1.5 > 0.3)
  // and gives 1.5 from the 11-beam near cluster. The plain-median value is still checked via
  // the median() helper so the edge-leakage effect stays documented.
  fu::Params full = defaultParams();
  full.sector_fraction = 1.0;
  const auto r_full =
      fu::fuse(u_min, u_max, simCamera(), forwardCamera(), tb3Scan(), ranges, 0.0, full);
  ASSERT_TRUE(r_full.ok());
  EXPECT_EQ(r_full.beams_in_sector, 23U);
  EXPECT_EQ(r_full.cluster_beams, 11U);
  EXPECT_NEAR(r_full.range, 1.5, kRangeTol);
  std::vector<double> plain(11, 1.5);
  plain.insert(plain.end(), 12, 3.0);
  EXPECT_DOUBLE_EQ(fu::median(plain), 3.0);
}

TEST(FusionMedian, OutliersRejectedAndEvenCountAveraged) {
  // Sector beams -6..6 deg (13). Scan limits [0.12, 3.5]; params [0.2, 3.0] -> effective
  // [max(0.12,0.2), min(3.5,3.0)] = [0.2, 3.0].
  std::vector<float> ranges(kBeams, kWall);
  ranges[beamAt(-6)] = kInf;   // inf
  ranges[beamAt(-5)] = kNaN;   // NaN
  ranges[beamAt(-4)] = 0.0F;   // 0.0 (no return)
  ranges[beamAt(-3)] = 0.05F;  // below scan.range_min 0.12
  ranges[beamAt(-2)] = 3.6F;   // above scan.range_max 3.5
  ranges[beamAt(-1)] = 0.15F;  // above scan min but below params.min_range 0.2
  ranges[beamAt(0)] = 3.2F;    // below scan max but above params.max_range 3.0
  ranges[beamAt(1)] = 1.4F;
  ranges[beamAt(2)] = 1.5F;
  ranges[beamAt(3)] = 1.6F;
  ranges[beamAt(4)] = 1.7F;
  ranges[beamAt(5)] = 1.55F;
  ranges[beamAt(6)] = 1.42F;
  // Valid: {1.4, 1.42, 1.5, 1.55, 1.6, 1.7} (6, even) -> median (1.5 + 1.55) / 2 = 1.525.
  const fu::Params p{0.6, 0.2, 3.0};
  const auto r = fu::fuse(uForYaw(0.2), uForYaw(-0.2), simCamera(), forwardCamera(), tb3Scan(),
                          ranges, 0.0, p);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.beams_in_sector, 13U);
  EXPECT_EQ(r.valid_beams, 6U);
  EXPECT_NEAR(r.range, 1.525, kRangeTol);
}

TEST(FusionMedian, MedianHelper) {
  std::vector<double> odd{3.0, 1.0, 2.0};
  EXPECT_DOUBLE_EQ(fu::median(odd), 2.0);
  std::vector<double> even{4.0, 1.0, 3.0, 2.0};
  EXPECT_DOUBLE_EQ(fu::median(even), 2.5);
  std::vector<double> one{7.0};
  EXPECT_DOUBLE_EQ(fu::median(one), 7.0);
}

// ---------------------------------------------------------------- wrap-around

TEST(FusionWrap, SectorStraddlingZeroUsesBothEndsOfScan) {
  // TB3 scan 0..2pi: forward sector -6..6 deg = indices 0..6 and 354..359.
  // Everything outside is inf (invalid), so a correct result needs beams from both ends.
  std::vector<float> ranges(kBeams, kInf);
  for (int d = 0; d <= 6; ++d) {
    ranges[beamAt(d)] = 1.0F;  // 7 beams, low indices
  }
  for (int d = -6; d <= -1; ++d) {
    ranges[beamAt(d)] = 2.0F;  // 6 beams, indices 354..359
  }
  const auto r = fu::fuse(uForYaw(0.2), uForYaw(-0.2), simCamera(), forwardCamera(), tb3Scan(),
                          ranges, 0.0, defaultParams());
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.beams_in_sector, 13U);
  EXPECT_EQ(r.valid_beams, 13U);
  // Sorted: 7 x 1.0 then 6 x 2.0 -> 7th value = 1.0.
  EXPECT_NEAR(r.range, 1.0, kRangeTol);
}

TEST(FusionWrap, SectorStraddlingPiWithMinusPiToPiScan) {
  // Backward-looking camera (R = Rz(pi) * R0), scan angle -pi..pi: the sector around pi
  // straddles the end/start of this scan. Centred bbox -> bearing pi (or -pi).
  const Mat3 rot = matMul(rotZ(kPi), kOpticalToLaser);
  const fu::ScanGeometry scan{-kPi, 2.0 * kPi / static_cast<double>(kBeams), kScanRangeMin,
                              kScanRangeMax};
  // Beam i at -pi + i deg; the beams within +-6 deg of pi are i = 0..6 and 354..359.
  std::vector<float> ranges(kBeams, kInf);
  for (std::size_t i = 0; i <= 6; ++i) {
    ranges[i] = 2.0F;
  }
  for (std::size_t i = 354; i < kBeams; ++i) {
    ranges[i] = 2.0F;
  }
  const auto r = fu::fuse(uForYaw(0.2), uForYaw(-0.2), simCamera(), fu::Transform{rot, {0, 0, 0}},
                          scan, ranges, 0.5, defaultParams());
  ASSERT_TRUE(r.ok()) << fu::toString(r.status);
  EXPECT_NEAR(std::abs(r.bearing), kPi, kTol);
  EXPECT_EQ(r.beams_in_sector, 13U);
  // Position = (2.0 + 0.5) * (cos pi, sin pi) = (-2.5, 0).
  EXPECT_NEAR(r.x, -2.5, kRangeTol);
  EXPECT_NEAR(r.y, 0.0, kRangeTol);
}

TEST(FusionWrap, NegativeAngleIncrement) {
  // Same physical scan listed clockwise: angle_min = 2pi, increment -1 deg. Beam i is at
  // 2pi - i deg, so forward +-6 deg = indices 0..6 and 354..359 again.
  const fu::ScanGeometry scan{2.0 * kPi, -2.0 * kPi / static_cast<double>(kBeams), kScanRangeMin,
                              kScanRangeMax};
  std::vector<float> ranges(kBeams, kInf);
  for (std::size_t i = 0; i <= 6; ++i) {
    ranges[i] = 1.5F;
  }
  for (std::size_t i = 354; i < kBeams; ++i) {
    ranges[i] = 1.5F;
  }
  const auto r = fu::fuse(uForYaw(0.2), uForYaw(-0.2), simCamera(), forwardCamera(), scan, ranges,
                          0.0, defaultParams());
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.beams_in_sector, 13U);
  EXPECT_NEAR(r.range, 1.5, kRangeTol);
}

// ---------------------------------------------------------------- push-out (D-10)

TEST(FusionPushOut, MedianPlusRadiusAlongBearing) {
  // Median 2.0 at bearing 0, radius 0.35 -> (2.0 + 0.35) * (cos 0, sin 0) = (2.35, 0).
  // `range` reports the LiDAR median before the push-out.
  const std::vector<float> ranges(kBeams, 2.0F);
  const auto r = fu::fuse(kCx - 50.0, kCx + 50.0, simCamera(), forwardCamera(), tb3Scan(), ranges,
                          0.35, defaultParams());
  ASSERT_TRUE(r.ok());
  EXPECT_NEAR(r.range, 2.0, kRangeTol);
  EXPECT_NEAR(r.x, 2.35, kRangeTol);
  EXPECT_NEAR(r.y, 0.0, kRangeTol);
}

TEST(FusionPushOut, FractionOfRadius) {
  // Median 2.0 at bearing 0, radius 0.35, push_out_fraction 0.5 -> (2.0 + 0.175, 0) = (2.175, 0);
  // fraction 0 -> the LiDAR point itself (2.0, 0). `range` is unchanged (2.0) in both cases.
  const std::vector<float> ranges(kBeams, 2.0F);
  fu::Params half = defaultParams();
  half.push_out_fraction = 0.5;
  const auto r =
      fu::fuse(kCx - 50.0, kCx + 50.0, simCamera(), forwardCamera(), tb3Scan(), ranges, 0.35, half);
  ASSERT_TRUE(r.ok());
  EXPECT_NEAR(r.range, 2.0, kRangeTol);
  EXPECT_NEAR(r.x, 2.175, kRangeTol);
  fu::Params none = defaultParams();
  none.push_out_fraction = 0.0;
  const auto r0 =
      fu::fuse(kCx - 50.0, kCx + 50.0, simCamera(), forwardCamera(), tb3Scan(), ranges, 0.35, none);
  ASSERT_TRUE(r0.ok());
  EXPECT_NEAR(r0.x, 2.0, kRangeTol);
}

TEST(FusionPushOut, FractionOutsideUnitIntervalThrows) {
  fu::Params p = defaultParams();
  p.push_out_fraction = -0.1;
  EXPECT_THROW(fu::validate(p), std::invalid_argument);
  p.push_out_fraction = 1.1;
  EXPECT_THROW(fu::validate(p), std::invalid_argument);
  p.push_out_fraction = 1.0;
  EXPECT_NO_THROW(fu::validate(p));
}

// ---------------------------------------------------------------- failures

TEST(FusionFailure, AllInvalidGivesNoValidRanges) {
  std::vector<float> ranges(kBeams, kInf);
  ranges[beamAt(0)] = kNaN;
  ranges[beamAt(1)] = 0.0F;
  const auto r = fu::fuse(uForYaw(0.2), uForYaw(-0.2), simCamera(), forwardCamera(), tb3Scan(),
                          ranges, 0.3, defaultParams());
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.status, fu::Status::NoValidRanges);
  EXPECT_EQ(r.beams_in_sector, 13U);
  EXPECT_EQ(r.valid_beams, 0U);
  EXPECT_EQ(r.x, 0.0);
  EXPECT_EQ(r.y, 0.0);
}

TEST(FusionFailure, EmptySector) {
  // Empty ranges array -> no beam in any sector.
  const auto r = fu::fuse(kCx - 50.0, kCx + 50.0, simCamera(), forwardCamera(), tb3Scan(), {}, 0.0,
                          defaultParams());
  EXPECT_EQ(r.status, fu::Status::EmptySector);
  // Very narrow bbox between two beams: u_c = cx + fx*tan(0.5 deg) -> bearing -0.5 deg;
  // bbox +-0.5 px -> sector half-width ~0.0005 rad * 0.6, no beam (beams at whole degrees).
  const double uc = kCx + kFx * std::tan(0.5 * kDeg);
  const std::vector<float> ranges(kBeams, kWall);
  const auto r2 = fu::fuse(uc - 0.5, uc + 0.5, simCamera(), forwardCamera(), tb3Scan(), ranges, 0.0,
                           defaultParams());
  EXPECT_EQ(r2.status, fu::Status::EmptySector);
  EXPECT_EQ(r2.beams_in_sector, 0U);
}

TEST(FusionFailure, BadInputs) {
  const std::vector<float> ranges(kBeams, kWall);
  const auto cam = simCamera();
  const auto tf = forwardCamera();
  const auto scan = tb3Scan();
  const auto p = defaultParams();
  // u_min >= u_max
  EXPECT_EQ(fu::fuse(300, 300, cam, tf, scan, ranges, 0.0, p).status, fu::Status::BadInput);
  EXPECT_EQ(fu::fuse(310, 300, cam, tf, scan, ranges, 0.0, p).status, fu::Status::BadInput);
  // non-finite bbox
  EXPECT_EQ(fu::fuse(std::nan(""), 300, cam, tf, scan, ranges, 0.0, p).status,
            fu::Status::BadInput);
  // fx <= 0 (CameraInfo not received yet -> K all zero)
  EXPECT_EQ(fu::fuse(200, 300, fu::Intrinsics{0.0, kCx}, tf, scan, ranges, 0.0, p).status,
            fu::Status::BadInput);
  // zero angle increment
  fu::ScanGeometry bad_scan = scan;
  bad_scan.angle_increment = 0.0;
  EXPECT_EQ(fu::fuse(200, 300, cam, tf, bad_scan, ranges, 0.0, p).status, fu::Status::BadInput);
  // negative radius
  EXPECT_EQ(fu::fuse(200, 300, cam, tf, scan, ranges, -0.1, p).status, fu::Status::BadInput);
  // degenerate rotation whose rows 0-1 vanish: every ray is vertical in the laser frame
  const fu::Transform vertical{{0, 0, 0, 0, 0, 0, 1, 0, 1}, {0, 0, 0}};
  EXPECT_EQ(fu::fuse(200, 300, cam, vertical, scan, ranges, 0.0, p).status, fu::Status::BadInput);
}

TEST(FusionFailure, BadParamsThrow) {
  EXPECT_THROW(fu::validate(fu::Params{0.0, 0.0, 10.0}), std::invalid_argument);
  EXPECT_THROW(fu::validate(fu::Params{-0.5, 0.0, 10.0}), std::invalid_argument);
  EXPECT_THROW(fu::validate(fu::Params{1.01, 0.0, 10.0}), std::invalid_argument);
  EXPECT_THROW(fu::validate(fu::Params{std::nan(""), 0.0, 10.0}), std::invalid_argument);
  EXPECT_THROW(fu::validate(fu::Params{0.6, 2.0, 2.0}), std::invalid_argument);
  EXPECT_THROW(fu::validate(fu::Params{0.6, 3.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(fu::validate(fu::Params{0.6, -0.1, 1.0}), std::invalid_argument);
  EXPECT_NO_THROW(fu::validate(fu::Params{1.0, 0.0, 3.5}));
  // fuse() validates too.
  const std::vector<float> ranges(kBeams, kWall);
  EXPECT_THROW(fu::fuse(200, 300, simCamera(), forwardCamera(), tb3Scan(), ranges, 0.0,
                        fu::Params{0.0, 0.0, 10.0}),
               std::invalid_argument);
}

// ---------------------------------------------------------------- nearest cluster (D-22)

namespace {
// Measured case (D-22, person at ~1.43 m): central-sector beams -5..5 deg, in beam order.
const std::vector<double> kLegsAndWall{2.78, 1.48, 1.45, 1.46, 1.47, 1.48,
                                       2.68, 2.68, 2.72, 2.75, 2.79};
}  // namespace

TEST(FusionCluster, LegsAndWallHelper) {
  // Sorted: 1.45 1.46 1.47 1.48 1.48 | 2.68 2.68 2.72 2.75 2.78 2.79.
  // Gap 2.68 - 1.48 = 1.20 > 0.3 -> clusters of 5 and 6. Nearest (5 >= 2): median = 3rd = 1.47.
  // Plain median of 11 values = 6th sorted = 2.68 (the wall).
  std::size_t n = 99;
  const auto m = fu::nearestClusterMedian(kLegsAndWall, 0.3, 2, &n);
  ASSERT_TRUE(m.has_value());
  EXPECT_NEAR(*m, 1.47, kTol);
  EXPECT_EQ(n, 5U);
  std::vector<double> copy = kLegsAndWall;
  EXPECT_NEAR(fu::median(copy), 2.68, kTol);
}

TEST(FusionCluster, LegsAndWallThroughFuse) {
  // Bbox half-width h = 5.5 deg / 0.6 = 9.1667 deg -> shrunk sector +-5.5 deg -> beams -5..5
  // (11). Values (float) as in the helper test -> range 1.47 (float tolerance), 5-beam cluster.
  // Position with radius 0.3, push_out_fraction 1.0: (1.47 + 0.3, 0) = (1.77, 0).
  std::vector<float> ranges(kBeams, kWall);
  for (int d = -5; d <= 5; ++d) {
    ranges[beamAt(d)] = static_cast<float>(kLegsAndWall[static_cast<std::size_t>(d + 5)]);
  }
  const double h = 5.5 * kDeg / 0.6;
  const auto r = fu::fuse(uForYaw(h), uForYaw(-h), simCamera(), forwardCamera(), tb3Scan(), ranges,
                          0.3, defaultParams());
  ASSERT_TRUE(r.ok()) << fu::toString(r.status);
  EXPECT_EQ(r.beams_in_sector, 11U);
  EXPECT_EQ(r.valid_beams, 11U);
  EXPECT_EQ(r.cluster_beams, 5U);
  EXPECT_NEAR(r.range, 1.47, kRangeTol);
  EXPECT_NEAR(r.x, 1.77, kRangeTol);
  EXPECT_NEAR(r.y, 0.0, kRangeTol);
}

TEST(FusionCluster, SingleNearOutlierSkipped) {
  // {0.5, 2.0 x 6}: gap 1.5 > 0.3 -> clusters {0.5} (1 beam < 2, skipped) and {2.0 x 6} -> 2.0.
  const std::vector<double> v{2.0, 2.0, 0.5, 2.0, 2.0, 2.0, 2.0};
  std::size_t n = 0;
  const auto m = fu::nearestClusterMedian(v, 0.3, 2, &n);
  ASSERT_TRUE(m.has_value());
  EXPECT_DOUBLE_EQ(*m, 2.0);
  EXPECT_EQ(n, 6U);
  // With min_cluster_beams 1 the lone beam IS the nearest cluster -> 0.5.
  EXPECT_DOUBLE_EQ(*fu::nearestClusterMedian(v, 0.3, 1), 0.5);

  // Through fuse(): sector beams -6..6 (13), beam 0 at 0.5, the other 12 at 2.0 -> 2.0.
  std::vector<float> ranges(kBeams, kWall);
  for (int d = -6; d <= 6; ++d) {
    ranges[beamAt(d)] = 2.0F;
  }
  ranges[beamAt(0)] = 0.5F;
  const auto r = fu::fuse(uForYaw(0.2), uForYaw(-0.2), simCamera(), forwardCamera(), tb3Scan(),
                          ranges, 0.0, defaultParams());
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.cluster_beams, 12U);
  EXPECT_NEAR(r.range, 2.0, kRangeTol);
}

TEST(FusionCluster, SingleClusterEqualsPlainMedian) {
  // {1.4, 1.42, 1.5, 1.55, 1.6, 1.7}: max consecutive gap 0.1 <= 0.3 -> one cluster of 6;
  // median (1.5 + 1.55) / 2 = 1.525, same as median(). Odd case {2.1, 2.0, 2.2} -> 2.1.
  const std::vector<double> even{1.6, 1.4, 1.7, 1.5, 1.42, 1.55};
  std::size_t n = 0;
  EXPECT_NEAR(*fu::nearestClusterMedian(even, 0.3, 2, &n), 1.525, kTol);
  EXPECT_EQ(n, 6U);
  std::vector<double> copy = even;
  EXPECT_NEAR(fu::median(copy), 1.525, kTol);
  EXPECT_NEAR(*fu::nearestClusterMedian({2.1, 2.0, 2.2}, 0.3, 2), 2.1, kTol);
}

TEST(FusionCluster, NoQualifyingClusterGivesNoCluster) {
  // {1.0, 1.5, 2.0, 2.5}: every consecutive gap 0.5 > 0.3 -> four 1-beam clusters; min 3 ->
  // none qualifies -> nullopt, cluster size 0. Empty input -> nullopt as well.
  std::size_t n = 99;
  EXPECT_FALSE(fu::nearestClusterMedian({1.0, 1.5, 2.0, 2.5}, 0.3, 3, &n).has_value());
  EXPECT_EQ(n, 0U);
  EXPECT_FALSE(fu::nearestClusterMedian({}, 0.3, 1).has_value());

  // Through fuse(): sector beams -6..6 (13), all inf except 4 beams at 1.0, 1.5, 2.0, 2.5;
  // min_cluster_beams 3 -> NoCluster, nothing guessed.
  std::vector<float> ranges(kBeams, kInf);
  ranges[beamAt(-3)] = 1.0F;
  ranges[beamAt(-1)] = 1.5F;
  ranges[beamAt(1)] = 2.0F;
  ranges[beamAt(3)] = 2.5F;
  fu::Params p = defaultParams();
  p.min_cluster_beams = 3;
  const auto r = fu::fuse(uForYaw(0.2), uForYaw(-0.2), simCamera(), forwardCamera(), tb3Scan(),
                          ranges, 0.3, p);
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.status, fu::Status::NoCluster);
  EXPECT_STREQ(fu::toString(r.status), "NoCluster");
  EXPECT_EQ(r.beams_in_sector, 13U);
  EXPECT_EQ(r.valid_beams, 4U);
  EXPECT_EQ(r.cluster_beams, 0U);
  EXPECT_EQ(r.range, 0.0);
  EXPECT_EQ(r.x, 0.0);
  EXPECT_EQ(r.y, 0.0);
}

TEST(FusionCluster, GapBoundaryIsInclusive) {
  // Split rule: new cluster only if difference > gap; difference == gap stays together.
  // Values are exact binary fractions so the difference is exactly the gap.
  // {1.0, 1.5}, gap 0.5: 1.5 - 1.0 = 0.5, not > 0.5 -> one cluster of 2 -> median 1.25.
  std::size_t n = 0;
  EXPECT_DOUBLE_EQ(*fu::nearestClusterMedian({1.5, 1.0}, 0.5, 2, &n), 1.25);
  EXPECT_EQ(n, 2U);
  // Gap 0.25 < 0.5 -> split into {1.0}, {1.5}; min 2 -> none qualifies.
  EXPECT_FALSE(fu::nearestClusterMedian({1.5, 1.0}, 0.25, 2).has_value());
  // Chained: {1.0, 1.25, 1.5, 2.0}, gap 0.25: 0.25, 0.25 stay, 0.5 splits -> {1.0, 1.25, 1.5}
  // -> median 1.25, size 3.
  EXPECT_DOUBLE_EQ(*fu::nearestClusterMedian({2.0, 1.5, 1.25, 1.0}, 0.25, 2, &n), 1.25);
  EXPECT_EQ(n, 3U);
}

TEST(FusionCluster, BadClusterParamsThrow) {
  fu::Params p = defaultParams();
  EXPECT_DOUBLE_EQ(p.cluster_gap, 0.3);  // documented defaults
  EXPECT_EQ(p.min_cluster_beams, 2U);
  EXPECT_NO_THROW(fu::validate(p));
  p.cluster_gap = 0.0;
  EXPECT_THROW(fu::validate(p), std::invalid_argument);
  p.cluster_gap = -0.1;
  EXPECT_THROW(fu::validate(p), std::invalid_argument);
  p.cluster_gap = std::nan("");
  EXPECT_THROW(fu::validate(p), std::invalid_argument);
  p.cluster_gap = std::numeric_limits<double>::infinity();
  EXPECT_THROW(fu::validate(p), std::invalid_argument);
  p.cluster_gap = 0.3;
  p.min_cluster_beams = 0;
  EXPECT_THROW(fu::validate(p), std::invalid_argument);
  p.min_cluster_beams = 1;
  EXPECT_NO_THROW(fu::validate(p));
  // fuse() validates too.
  p.min_cluster_beams = 0;
  const std::vector<float> ranges(kBeams, kWall);
  EXPECT_THROW(fu::fuse(200, 300, simCamera(), forwardCamera(), tb3Scan(), ranges, 0.0, p),
               std::invalid_argument);
}
