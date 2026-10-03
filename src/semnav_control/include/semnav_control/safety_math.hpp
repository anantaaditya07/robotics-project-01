// Safety gate logic (architecture 7.4, D-13). ROS-free, header-only, plain STL.
// safety_gate_node only wires ROS messages, parameters and the 20 Hz timer to these functions.
//
// Per timer tick:
//  1) Watchdog: if the last /cmd_vel_nav is older than cmd_timeout, or the last /scan is older
//     than scan_timeout (or either was never received), the output is zero (linear AND angular).
//     A non-finite command is treated the same way. "Older than" is strict: age == timeout is
//     still fresh.
//  2) Cone: centred on scan angle 0 (forward) when linear.x >= 0, on scan angle pi (rearward)
//     when linear.x < 0 (D-13, accepted deviation: the PDF specifies a forward cone only).
//     Half-angle = cone_half_angle + cone_widen_gain * |angular.z|, capped at
//     cone_max_half_angle. The widened cone stays symmetric about its centre.
//  3) d = minimum VALID range inside the cone. A range is valid when it is finite and
//     range_min <= r <= range_max; everything else (NaN, +/-inf, out of limits) is ignored.
//     If the cone holds no valid range it is treated as clear (d = +inf -> s = 1): the TB3
//     LiDAR reports +inf for "no return within range_max", so an empty cone means nothing is
//     there. The scan watchdog still covers a dead LiDAR.
//  4) s = clamp((d - d_stop) / (d_slow - d_stop), 0, 1) (end points compared at the float32
//     precision of LaserScan ranges); target linear = s * linear.x;
//     angular.z passes through unchanged so the robot can still turn away.
//  5) Acceleration limiter on linear.x, max_delta_v per tick: INCREASES of |v| are limited;
//     DECREASES of |v| (braking, safety stop, watchdog stop) are applied immediately, so a
//     safety stop is never slowed down by the limiter. A sign reversal is treated as an
//     immediate drop to zero followed by a limited increase in the new direction.
//
// Scan angles: beam i is at angle_min + i * angle_increment. Angles are compared via a
// difference normalised to [-pi, pi], so cones straddling the 0 / 2*pi seam of the TurtleBot3
// scan (angle 0 .. 2*pi, D-07), scans in -pi .. pi, and negative increments all work.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace semnav_control::safety {

/// pi (M_PI is not standard C++).
constexpr double kPi = 3.14159265358979323846;

/// Absorbs floating point round-off when a beam lies exactly on the cone boundary.
/// Not a tunable.
constexpr double kAngleEpsilon = 1e-9;

/// Gate parameters, all in SI units (radians, metres, seconds, m/s).
struct GateParams {
  double cone_half_angle = 30.0 * kPi / 180.0;      ///< base half-angle (rad)
  double cone_widen_gain = 20.0 * kPi / 180.0;      ///< extra half-angle per rad/s (rad per rad/s)
  double cone_max_half_angle = 60.0 * kPi / 180.0;  ///< cap on the widened half-angle (rad)
  double d_stop = 0.3;                              ///< m, s = 0 at or below
  double d_slow = 0.6;                              ///< m, s = 1 at or above
  double cmd_timeout = 0.5;                         ///< s
  double scan_timeout = 0.3;                        ///< s
  double max_delta_v = 0.05;                        ///< m/s per tick (max_accel / rate_hz)
};

/// Throws std::invalid_argument if the parameters are inconsistent.
inline void validate(const GateParams& p) {
  if (!(p.cone_half_angle > 0.0 && p.cone_half_angle <= kPi)) {
    throw std::invalid_argument("cone_half_angle must be in (0, 180] deg");
  }
  if (!(p.cone_widen_gain >= 0.0) || !std::isfinite(p.cone_widen_gain)) {
    throw std::invalid_argument("cone_widen_gain must be finite and >= 0");
  }
  if (!(p.cone_max_half_angle >= p.cone_half_angle && p.cone_max_half_angle <= kPi)) {
    throw std::invalid_argument("cone_max_half_angle must be in [cone_half_angle, 180] deg");
  }
  if (!(p.d_stop >= 0.0) || !std::isfinite(p.d_slow) || !(p.d_stop < p.d_slow)) {
    throw std::invalid_argument("need 0 <= d_stop < d_slow (finite)");
  }
  if (!(p.cmd_timeout > 0.0) || !(p.scan_timeout > 0.0)) {
    throw std::invalid_argument("cmd_timeout and scan_timeout must be > 0");
  }
  if (!(p.max_delta_v > 0.0) || !std::isfinite(p.max_delta_v)) {
    throw std::invalid_argument("max_delta_v (max_accel / rate_hz) must be finite and > 0");
  }
}

/// Wraps an angle to [-pi, pi].
inline double normalize_angle(double a) { return std::remainder(a, 2.0 * kPi); }

/// Planar velocity command (the only Twist fields a differential drive uses).
struct Velocity {
  double linear = 0.0;   ///< linear.x, m/s
  double angular = 0.0;  ///< angular.z, rad/s
};

/// Read-only view of a LaserScan, so this header needs no ROS types.
struct ScanView {
  double angle_min = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
  const std::vector<float>* ranges = nullptr;
};

/// Cone in scan-frame angles.
struct Cone {
  double centre = 0.0;      ///< rad
  double half_angle = 0.0;  ///< rad
};

/// D-13: forward cone for linear >= 0, rear cone for linear < 0; widened with |angular|.
inline Cone cone_for_command(const Velocity& cmd, const GateParams& p) {
  Cone c;
  c.centre = cmd.linear < 0.0 ? kPi : 0.0;
  c.half_angle = std::min(p.cone_half_angle + p.cone_widen_gain * std::abs(cmd.angular),
                          p.cone_max_half_angle);
  return c;
}

/// Minimum valid range inside the cone, or +inf if the cone holds no valid range.
inline double cone_min_range(const ScanView& scan, const Cone& cone) {
  double d = std::numeric_limits<double>::infinity();
  if (scan.ranges == nullptr) {
    return d;
  }
  const auto& r = *scan.ranges;
  for (std::size_t i = 0; i < r.size(); ++i) {
    const double range = static_cast<double>(r[i]);
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (std::abs(normalize_angle(angle - cone.centre)) <= cone.half_angle + kAngleEpsilon) {
      d = std::min(d, range);
    }
  }
  return d;
}

/// s = clamp((d - d_stop) / (d_slow - d_stop), 0, 1); d = +inf gives 1. Requires d_stop < d_slow.
/// LaserScan ranges are float32, so the end points are compared at float32 precision: a range
/// that equals d_stop (or d_slow) as the LiDAR can represent it gives exactly 0 (or 1). Without
/// this, 0.3F = 0.30000001 > 0.3 would give s = 4e-8 instead of a stop.
inline double scale_factor(double d, double d_stop, double d_slow) {
  if (std::isnan(d)) {
    return 0.0;
  }
  if (std::isinf(d)) {
    return d > 0.0 ? 1.0 : 0.0;
  }
  if (d <= static_cast<double>(static_cast<float>(d_stop))) {
    return 0.0;
  }
  if (d >= static_cast<double>(static_cast<float>(d_slow))) {
    return 1.0;
  }
  return std::clamp((d - d_stop) / (d_slow - d_stop), 0.0, 1.0);
}

/// Limits increases of |v| to max_delta per tick; decreases of |v| are immediate.
/// A sign change drops to zero first, then increases (limited) in the new direction.
inline double limit_accel(double target, double last, double max_delta) {
  const double base = (target * last > 0.0) ? last : 0.0;
  if (std::abs(target) <= std::abs(base)) {
    return target;
  }
  const double step = std::min(std::abs(target) - std::abs(base), max_delta);
  return base + std::copysign(step, target);
}

/// Inputs to one gate tick. Ages are seconds since the last message (+inf if never received).
struct GateInput {
  Velocity cmd;
  double last_linear = 0.0;  ///< linear.x published on the previous tick
  double cmd_age = std::numeric_limits<double>::infinity();
  double scan_age = std::numeric_limits<double>::infinity();
  double d_min = std::numeric_limits<double>::infinity();  ///< cone_min_range for this cmd's cone
};

/// Why the output differs from the command (for logging only).
enum class GateState { kPass, kScaled, kStopped, kStaleCmd, kStaleScan, kInvalidCmd };

struct GateOutput {
  Velocity vel;
  GateState state = GateState::kPass;
  double scale = 1.0;
};

/// One gate tick: watchdog, scaling, acceleration limiter.
inline GateOutput gate_step(const GateInput& in, const GateParams& p) {
  GateOutput out;
  if (!(in.cmd_age <= p.cmd_timeout)) {
    out.state = GateState::kStaleCmd;
    out.scale = 0.0;
    return out;
  }
  if (!(in.scan_age <= p.scan_timeout)) {
    out.state = GateState::kStaleScan;
    out.scale = 0.0;
    return out;
  }
  if (!std::isfinite(in.cmd.linear) || !std::isfinite(in.cmd.angular)) {
    out.state = GateState::kInvalidCmd;
    out.scale = 0.0;
    return out;
  }
  out.scale = scale_factor(in.d_min, p.d_stop, p.d_slow);
  out.state = out.scale >= 1.0 ? GateState::kPass
                               : (out.scale <= 0.0 ? GateState::kStopped : GateState::kScaled);
  out.vel.linear = limit_accel(out.scale * in.cmd.linear, in.last_linear, p.max_delta_v);
  out.vel.angular = in.cmd.angular;
  return out;
}

/// Convenience: cone selection + cone minimum + gate_step in one call (used by the node and
/// the tests). scan may have ranges == nullptr when no scan was ever received.
inline GateOutput gate_step(const Velocity& cmd, double last_linear, double cmd_age,
                            double scan_age, const ScanView& scan, const GateParams& p) {
  GateInput in;
  in.cmd = cmd;
  in.last_linear = last_linear;
  in.cmd_age = cmd_age;
  in.scan_age = scan_age;
  in.d_min = cone_min_range(scan, cone_for_command(cmd, p));
  return gate_step(in, p);
}

}  // namespace semnav_control::safety
