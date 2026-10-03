// Unit tests for safety_math.hpp (architecture 7.4 tests, D-13 required tests, D-07 wrap-around).
// All expected values are hand-computed in the comments.
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <semnav_control/safety_math.hpp>
#include <stdexcept>
#include <vector>

namespace s = semnav_control::safety;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDeg = s::kPi / 180.0;
constexpr std::size_t kBeams = 360;  // TB3: 360 samples, 1 deg apart (D-07)
// TB3 LiDAR limits (D-07); float32 like the LaserScan fields they come from.
constexpr float kRangeMin = 0.12F;
constexpr float kRangeMax = 3.5F;
constexpr double kFresh = 0.05;  // s, an age well inside both timeouts
constexpr double kTol = 1e-12;

// Owns the ranges a ScanView points to.
struct Scan {
  double angle_min = 0.0;
  double increment = 0.0;
  std::vector<float> ranges;

  s::ScanView view() const {
    s::ScanView v;
    v.angle_min = angle_min;
    v.angle_increment = increment;
    v.range_min = kRangeMin;
    v.range_max = kRangeMax;
    v.ranges = &ranges;
    return v;
  }

  // Index of the beam closest to angle_deg (any representation of the angle).
  std::size_t index_of(double angle_deg) const {
    std::size_t best = 0;
    double best_err = kInf;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      const double a = angle_min + static_cast<double>(i) * increment;
      const double err = std::abs(s::normalize_angle(a - angle_deg * kDeg));
      if (err < best_err) {
        best_err = err;
        best = i;
      }
    }
    return best;
  }

  void set(double angle_deg, float r) { ranges[index_of(angle_deg)] = r; }

  // Sets every beam with angle in [from_deg, to_deg] (stepping 1 deg) to r.
  void set_span(int from_deg, int to_deg, float r) {
    for (int a = from_deg; a <= to_deg; ++a) {
      set(a, r);
    }
  }
};

// TurtleBot3 convention: angle 0 .. 2*pi, +1 deg (D-07). Everything "no return" (+inf).
Scan tb3_scan() {
  return Scan{0.0, 2.0 * s::kPi / kBeams, std::vector<float>(kBeams, static_cast<float>(kInf))};
}

// -pi .. pi convention, +1 deg.
Scan centred_scan() {
  return Scan{-s::kPi, 2.0 * s::kPi / kBeams, std::vector<float>(kBeams, static_cast<float>(kInf))};
}

// Negative increment: pi down to -pi + 1 deg.
Scan reversed_scan() {
  return Scan{s::kPi, -2.0 * s::kPi / kBeams, std::vector<float>(kBeams, static_cast<float>(kInf))};
}

s::GateOutput run(const Scan& scan, double v, double w, double last, const s::GateParams& p = {},
                  double cmd_age = kFresh, double scan_age = kFresh) {
  return s::gate_step(s::Velocity{v, w}, last, cmd_age, scan_age, scan.view(), p);
}

}  // namespace

// ---------------------------------------------------------------- PDF 7.4 tests (defaults)

TEST(SafetyGate, DefaultsAreValid) { EXPECT_NO_THROW(s::validate(s::GateParams{})); }

// Wall at 0.3 m across the forward cone; default d_stop 0.3 -> s = 0 -> linear 0.
// (0.3F = 0.30000001 > 0.3: zero relies on the float32 end-point comparison in scale_factor.)
TEST(SafetyGate, WallAt0p3InFrontGivesZeroLinear) {
  auto scan = tb3_scan();
  scan.set_span(-45, 45, 0.3F);
  const auto out = run(scan, 0.2, 0.0, 0.2);
  EXPECT_EQ(out.vel.linear, 0.0);
  EXPECT_EQ(out.state, s::GateState::kStopped);
}

// Wall at 0.3 m with a larger d_stop (0.35) -> still zero.
TEST(SafetyGate, WallInsideLargerDStopGivesZero) {
  s::GateParams p;
  p.d_stop = 0.35;
  auto scan = tb3_scan();
  scan.set_span(-45, 45, 0.3F);
  EXPECT_EQ(run(scan, 0.2, 0.0, 0.2, p).vel.linear, 0.0);
}

// Everything at 3.0 m (> d_slow) -> s = 1 -> exact passthrough of both components.
TEST(SafetyGate, FarScanPassesThrough) {
  auto scan = tb3_scan();
  scan.ranges.assign(kBeams, 3.0F);
  const auto out = run(scan, 0.2, 0.5, 0.2);
  EXPECT_EQ(out.vel.linear, 0.2);
  EXPECT_EQ(out.vel.angular, 0.5);
  EXPECT_EQ(out.state, s::GateState::kPass);
}

// d_stop 0.25, d_slow 0.75, wall at 0.5 (all exact in binary):
// s = (0.5 - 0.25) / 0.5 = 0.5 -> 0.5 * 0.2 = 0.1 (a decrease, so the limiter does not act).
TEST(SafetyGate, BetweenStopAndSlowScalesExactly) {
  s::GateParams p;
  p.d_stop = 0.25;
  p.d_slow = 0.75;
  auto scan = tb3_scan();
  scan.set(0, 0.5F);
  const auto out = run(scan, 0.2, 0.0, 0.2, p);
  EXPECT_DOUBLE_EQ(out.scale, 0.5);
  EXPECT_DOUBLE_EQ(out.vel.linear, 0.1);
  EXPECT_EQ(out.state, s::GateState::kScaled);
}

// Defaults (0.3 / 0.6), wall at 0.375 (exact in float): s = 0.075 / 0.3 = 0.25 -> 0.05.
TEST(SafetyGate, BetweenStopAndSlowScalesWithDefaults) {
  auto scan = tb3_scan();
  scan.set(10, 0.375F);
  const auto out = run(scan, 0.2, 0.0, 0.2);
  EXPECT_NEAR(out.scale, 0.25, kTol);
  EXPECT_NEAR(out.vel.linear, 0.05, kTol);
}

// Stopped by an obstacle: angular.z still passes so the robot can turn away.
TEST(SafetyGate, AngularPassesThroughWhenStopped) {
  auto scan = tb3_scan();
  scan.set(0, 0.2F);
  const auto out = run(scan, 0.2, 0.7, 0.2);
  EXPECT_EQ(out.vel.linear, 0.0);
  EXPECT_EQ(out.vel.angular, 0.7);
  EXPECT_EQ(out.state, s::GateState::kStopped);
}

// ---------------------------------------------------------------- watchdog

TEST(SafetyGate, StaleScanGivesZero) {
  auto scan = tb3_scan();
  scan.ranges.assign(kBeams, 3.0F);
  const auto stale = run(scan, 0.2, 0.5, 0.2, {}, kFresh, 0.31);
  EXPECT_EQ(stale.vel.linear, 0.0);
  EXPECT_EQ(stale.vel.angular, 0.0);
  EXPECT_EQ(stale.state, s::GateState::kStaleScan);
  // Exactly at the timeout is still fresh (strict ">").
  EXPECT_EQ(run(scan, 0.2, 0.5, 0.2, {}, kFresh, 0.3).vel.linear, 0.2);
}

TEST(SafetyGate, NoScanEverGivesZero) {
  const s::ScanView none;  // ranges == nullptr
  const auto out = s::gate_step(s::Velocity{0.2, 0.5}, 0.2, kFresh, kInf, none, s::GateParams{});
  EXPECT_EQ(out.vel.linear, 0.0);
  EXPECT_EQ(out.vel.angular, 0.0);
  EXPECT_EQ(out.state, s::GateState::kStaleScan);
}

TEST(SafetyGate, StaleCmdGivesZero) {
  auto scan = tb3_scan();
  scan.ranges.assign(kBeams, 3.0F);
  const auto stale = run(scan, 0.2, 0.5, 0.2, {}, 0.51, kFresh);
  EXPECT_EQ(stale.vel.linear, 0.0);
  EXPECT_EQ(stale.vel.angular, 0.0);
  EXPECT_EQ(stale.state, s::GateState::kStaleCmd);
  EXPECT_EQ(run(scan, 0.2, 0.5, 0.2, {}, kInf, kFresh).state, s::GateState::kStaleCmd);
  EXPECT_EQ(run(scan, 0.2, 0.5, 0.2, {}, 0.5, kFresh).vel.linear, 0.2);
}

TEST(SafetyGate, NonFiniteCmdGivesZero) {
  auto scan = tb3_scan();
  const auto out = run(scan, std::nan(""), 0.5, 0.2);
  EXPECT_EQ(out.vel.linear, 0.0);
  EXPECT_EQ(out.vel.angular, 0.0);
  EXPECT_EQ(out.state, s::GateState::kInvalidCmd);
}

// ---------------------------------------------------------------- D-13 cone direction

// Obstacle 0.3 m BEHIND (150..210 deg) + reverse command -> zero linear.
TEST(SafetyGateD13, ObstacleBehindReverseGivesZero) {
  auto scan = tb3_scan();
  scan.set_span(150, 210, 0.3F);
  const auto out = run(scan, -0.1, 0.0, -0.1);
  EXPECT_EQ(out.vel.linear, 0.0);
  EXPECT_EQ(out.state, s::GateState::kStopped);
}

// Same obstacle behind + forward command -> passthrough.
TEST(SafetyGateD13, ObstacleBehindForwardPassesThrough) {
  auto scan = tb3_scan();
  scan.set_span(150, 210, 0.3F);
  EXPECT_EQ(run(scan, 0.2, 0.0, 0.2).vel.linear, 0.2);
}

// Obstacle 0.3 m in front + reverse command -> passthrough.
TEST(SafetyGateD13, ObstacleInFrontReversePassesThrough) {
  auto scan = tb3_scan();
  scan.set_span(-30, 30, 0.3F);
  const auto out = run(scan, -0.1, 0.0, -0.1);
  EXPECT_EQ(out.vel.linear, -0.1);
  EXPECT_EQ(out.state, s::GateState::kPass);
}

TEST(SafetyGateD13, ConeCentreFollowsSignOfLinear) {
  const s::GateParams p;
  EXPECT_EQ(s::cone_for_command({0.1, 0.0}, p).centre, 0.0);
  EXPECT_EQ(s::cone_for_command({0.0, 0.0}, p).centre, 0.0);
  EXPECT_EQ(s::cone_for_command({-0.1, 0.0}, p).centre, s::kPi);
}

// ---------------------------------------------------------------- cone widening

// Defaults: 30 deg + 20 deg per rad/s. Single obstacle at +40 deg, 0.2 m.
// w = 0   -> half 30 deg -> not in cone -> passthrough.
// w = 0.6 -> half 42 deg -> in cone -> stop.
TEST(SafetyGateCone, WideningCatchesObstacleAt40DegOnlyWhenTurning) {
  auto scan = tb3_scan();
  scan.set(40, 0.2F);
  EXPECT_EQ(run(scan, 0.2, 0.0, 0.2).vel.linear, 0.2);
  EXPECT_EQ(run(scan, 0.2, 0.6, 0.2).vel.linear, 0.0);
  // Symmetric: turning the other way (w < 0) widens the same amount.
  EXPECT_EQ(run(scan, 0.2, -0.6, 0.2).vel.linear, 0.0);
  // w = 0.4 -> half 38 deg -> still outside.
  EXPECT_EQ(run(scan, 0.2, 0.4, 0.2).vel.linear, 0.2);
}

// Cap: w = 10 rad/s would give 230 deg; capped at 60 deg. Obstacle at 70 deg stays outside.
TEST(SafetyGateCone, WideningIsCapped) {
  const s::GateParams p;
  EXPECT_NEAR(s::cone_for_command({0.2, 10.0}, p).half_angle, 60.0 * kDeg, kTol);
  EXPECT_NEAR(s::cone_for_command({0.2, 1.0}, p).half_angle, 50.0 * kDeg, kTol);
  auto scan = tb3_scan();
  scan.set(70, 0.2F);
  EXPECT_EQ(run(scan, 0.2, 10.0, 0.2).vel.linear, 0.2);
  scan.set(55, 0.2F);
  EXPECT_EQ(run(scan, 0.2, 10.0, 0.2).vel.linear, 0.0);
}

// A beam exactly on the cone boundary (30 deg) is inside.
TEST(SafetyGateCone, BoundaryBeamIsInside) {
  auto scan = tb3_scan();
  scan.set(30, 0.2F);
  EXPECT_EQ(run(scan, 0.2, 0.0, 0.2).vel.linear, 0.0);
  auto scan2 = tb3_scan();
  scan2.set(31, 0.2F);
  EXPECT_EQ(run(scan2, 0.2, 0.0, 0.2).vel.linear, 0.2);
}

// ---------------------------------------------------------------- wrap-around (D-07)

// cone_min_range returns the exact minimum over the cone, across the 0 / 2*pi seam.
TEST(SafetyGateWrap, Tb3ForwardConeAcrossSeam) {
  auto scan = tb3_scan();
  scan.set(355, 1.0F);  // = -5 deg, beam index 355
  scan.set(3, 1.5F);
  scan.set(90, 0.5F);  // outside the cone
  EXPECT_EQ(scan.index_of(355), 355U);
  EXPECT_DOUBLE_EQ(s::cone_min_range(scan.view(), s::Cone{0.0, 30.0 * kDeg}), 1.0);
  // Through the gate: obstacle at -5 deg, 0.2 m -> stop.
  scan.set(355, 0.2F);
  EXPECT_EQ(run(scan, 0.2, 0.0, 0.2).vel.linear, 0.0);
}

TEST(SafetyGateWrap, Tb3RearCone) {
  auto scan = tb3_scan();
  scan.set(185, 0.2F);
  EXPECT_EQ(run(scan, -0.1, 0.0, -0.1).vel.linear, 0.0);
  EXPECT_EQ(run(scan, 0.1, 0.0, 0.1).vel.linear, 0.1);
}

// -pi .. pi: forward cone is in the middle of the array, rear cone straddles both ends.
TEST(SafetyGateWrap, CentredScanForwardAndRear) {
  auto scan = centred_scan();
  scan.set(-5, 0.2F);
  EXPECT_EQ(scan.index_of(-5), 175U);
  EXPECT_EQ(run(scan, 0.2, 0.0, 0.2).vel.linear, 0.0);
  EXPECT_EQ(run(scan, -0.1, 0.0, -0.1).vel.linear, -0.1);

  auto rear = centred_scan();
  rear.set(180, 0.2F);  // index 0 (angle -pi)
  EXPECT_EQ(rear.index_of(180), 0U);
  EXPECT_EQ(run(rear, -0.1, 0.0, -0.1).vel.linear, 0.0);
  auto rear2 = centred_scan();
  rear2.set(175, 0.2F);  // index 355
  EXPECT_EQ(run(rear2, -0.1, 0.0, -0.1).vel.linear, 0.0);
  EXPECT_EQ(run(rear2, 0.1, 0.0, 0.1).vel.linear, 0.1);
}

// Negative angle_increment (pi .. -pi).
TEST(SafetyGateWrap, NegativeIncrement) {
  auto scan = reversed_scan();
  scan.set(10, 0.2F);  // index 170
  EXPECT_EQ(scan.index_of(10), 170U);
  EXPECT_EQ(run(scan, 0.2, 0.0, 0.2).vel.linear, 0.0);
  EXPECT_EQ(run(scan, -0.1, 0.0, -0.1).vel.linear, -0.1);

  auto rear = reversed_scan();
  rear.set(-170, 0.2F);
  EXPECT_EQ(run(rear, -0.1, 0.0, -0.1).vel.linear, 0.0);
  EXPECT_EQ(run(rear, 0.1, 0.0, 0.1).vel.linear, 0.1);
}

// ---------------------------------------------------------------- invalid ranges

// NaN, +inf, -inf, below range_min, above range_max: all ignored -> cone clear -> passthrough.
TEST(SafetyGateRanges, InvalidRangesIgnored) {
  auto scan = tb3_scan();
  scan.set(-2, std::nanf(""));
  scan.set(-1, -std::numeric_limits<float>::infinity());
  scan.set(0, std::numeric_limits<float>::infinity());
  scan.set(1, 0.05F);  // < range_min 0.12
  scan.set(2, 4.0F);   // > range_max 3.5
  scan.set(3, 0.0F);   // < range_min
  EXPECT_EQ(s::cone_min_range(scan.view(), s::Cone{0.0, 30.0 * kDeg}), kInf);
  const auto out = run(scan, 0.2, 0.0, 0.2);
  EXPECT_EQ(out.vel.linear, 0.2);
  EXPECT_EQ(out.state, s::GateState::kPass);
  // One valid beam among the invalid ones is used.
  scan.set(4, 2.0F);
  EXPECT_DOUBLE_EQ(s::cone_min_range(scan.view(), s::Cone{0.0, 30.0 * kDeg}), 2.0);
}

// Limits themselves are valid: range_min (0.12) is a real reading and stops the robot.
TEST(SafetyGateRanges, RangeAtLimitsIsValid) {
  auto scan = tb3_scan();
  scan.set(0, kRangeMin);
  EXPECT_EQ(run(scan, 0.2, 0.0, 0.2).vel.linear, 0.0);
}

TEST(SafetyGateRanges, EmptyScanIsClear) {
  Scan scan{0.0, 1.0 * kDeg, {}};
  EXPECT_EQ(s::cone_min_range(scan.view(), s::Cone{0.0, 30.0 * kDeg}), kInf);
}

// ---------------------------------------------------------------- scale factor

TEST(SafetyGateScale, ScaleFactorValues) {
  EXPECT_EQ(s::scale_factor(0.3, 0.3, 0.6), 0.0);
  EXPECT_EQ(s::scale_factor(static_cast<double>(0.3F), 0.3, 0.6), 0.0);  // float32 LiDAR value
  EXPECT_EQ(s::scale_factor(static_cast<double>(0.6F), 0.3, 0.6), 1.0);
  EXPECT_GT(s::scale_factor(0.301, 0.3, 0.6), 0.0);
  EXPECT_EQ(s::scale_factor(0.1, 0.3, 0.6), 0.0);
  EXPECT_EQ(s::scale_factor(0.6, 0.3, 0.6), 1.0);
  EXPECT_EQ(s::scale_factor(2.0, 0.3, 0.6), 1.0);
  EXPECT_DOUBLE_EQ(s::scale_factor(0.5, 0.25, 0.75), 0.5);
  EXPECT_EQ(s::scale_factor(kInf, 0.3, 0.6), 1.0);
  EXPECT_EQ(s::scale_factor(std::nan(""), 0.3, 0.6), 0.0);
}

// ---------------------------------------------------------------- acceleration limiter

// max_delta 0.05: 0 -> 0.2 ramps 0.05, 0.10, 0.15, 0.20, 0.20.
TEST(SafetyGateAccel, IncreasesAreRamped) {
  double v = 0.0;
  const double expected[] = {0.05, 0.10, 0.15, 0.20, 0.20};
  for (double e : expected) {
    v = s::limit_accel(0.2, v, 0.05);
    EXPECT_NEAR(v, e, kTol);
  }
  // Reverse direction ramps too: 0 -> -0.05.
  EXPECT_NEAR(s::limit_accel(-0.2, 0.0, 0.05), -0.05, kTol);
}

TEST(SafetyGateAccel, DecreasesAreImmediate) {
  EXPECT_EQ(s::limit_accel(0.0, 0.2, 0.05), 0.0);
  EXPECT_EQ(s::limit_accel(0.05, 0.2, 0.05), 0.05);
  EXPECT_EQ(s::limit_accel(-0.01, -0.1, 0.05), -0.01);
  // Sign reversal: immediate drop to 0, then a limited step: 0.2 -> -0.05.
  EXPECT_NEAR(s::limit_accel(-0.2, 0.2, 0.05), -0.05, kTol);
}

// Through the gate (defaults, 0.05 m/s per tick): clear cone ramps up; an obstacle that
// appears at full speed stops the robot on the very next tick.
TEST(SafetyGateAccel, GateRampsUpButStopsImmediately) {
  auto clear = tb3_scan();
  double last = 0.0;
  last = run(clear, 0.2, 0.0, last).vel.linear;
  EXPECT_NEAR(last, 0.05, kTol);
  last = run(clear, 0.2, 0.0, last).vel.linear;
  EXPECT_NEAR(last, 0.10, kTol);
  last = 0.2;  // at full speed
  auto blocked = tb3_scan();
  blocked.set(0, 0.25F);
  EXPECT_EQ(run(blocked, 0.2, 0.0, last).vel.linear, 0.0);
  // Watchdog stop is immediate as well.
  EXPECT_EQ(run(clear, 0.2, 0.0, last, {}, 0.6, kFresh).vel.linear, 0.0);
}

// ---------------------------------------------------------------- validation

TEST(SafetyGateParams, InvalidParamsThrow) {
  auto bad = [](auto mutate) {
    s::GateParams p;
    mutate(p);
    return p;
  };
  EXPECT_THROW(s::validate(bad([](s::GateParams& p) { p.d_stop = 0.6; })), std::invalid_argument);
  EXPECT_THROW(s::validate(bad([](s::GateParams& p) { p.d_stop = 0.7; })), std::invalid_argument);
  EXPECT_THROW(s::validate(bad([](s::GateParams& p) { p.d_stop = -0.1; })), std::invalid_argument);
  EXPECT_THROW(s::validate(bad([](s::GateParams& p) { p.cone_half_angle = 0.0; })),
               std::invalid_argument);
  EXPECT_THROW(s::validate(bad([](s::GateParams& p) { p.cone_max_half_angle = 20.0 * kDeg; })),
               std::invalid_argument);
  EXPECT_THROW(s::validate(bad([](s::GateParams& p) { p.cone_widen_gain = -1.0; })),
               std::invalid_argument);
  EXPECT_THROW(s::validate(bad([](s::GateParams& p) { p.cmd_timeout = 0.0; })),
               std::invalid_argument);
  EXPECT_THROW(s::validate(bad([](s::GateParams& p) { p.scan_timeout = -1.0; })),
               std::invalid_argument);
  EXPECT_THROW(s::validate(bad([](s::GateParams& p) { p.max_delta_v = 0.0; })),
               std::invalid_argument);
}
