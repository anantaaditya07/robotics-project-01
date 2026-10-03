// safety_gate_node (architecture 7.4, D-13): /cmd_vel_nav + /scan -> /cmd_vel.
//
// The callbacks only store the latest command and scan with their arrival time; /cmd_vel is
// published exclusively from a fixed-rate timer (rate_hz, default 20 Hz), never from a
// callback. All gate logic lives in safety_math.hpp (ROS-free, unit-tested).
//
// Timing: message ages are measured on a steady clock at arrival (not from header stamps and
// not in sim time), and the timer is a wall timer. Reason: the watchdog guards against messages
// that stop ARRIVING; with use_sim_time the ROS clock advances only as fast as /clock is
// published, which would quantise ages and the 20 Hz tick. Consequence: if the simulation
// runs much slower than real time (scan period 0.2 s sim at TB3 5 Hz), the 0.3 s scan watchdog
// may trip; scan_timeout is a parameter.
//
// Output: only linear.x and angular.z are set (differential drive); all other Twist fields are
// published as zero.
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <semnav_control/safety_math.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <stdexcept>
#include <utility>

namespace semnav_control {

namespace {
constexpr double kDegToRad = safety::kPi / 180.0;
}  // namespace

class SafetyGateNode : public rclcpp::Node {
 public:
  SafetyGateNode() : rclcpp::Node("safety_gate_node"), steady_clock_(RCL_STEADY_TIME) {
    const double rate_hz = declare_parameter<double>("rate_hz", 20.0);
    if (!(rate_hz > 0.0)) {
      throw std::invalid_argument("rate_hz must be > 0");
    }
    params_.cone_half_angle = declare_parameter<double>("cone_half_angle_deg", 30.0) * kDegToRad;
    params_.cone_widen_gain = declare_parameter<double>("cone_widen_gain", 20.0) * kDegToRad;
    params_.cone_max_half_angle =
        declare_parameter<double>("cone_max_half_angle_deg", 60.0) * kDegToRad;
    params_.d_stop = declare_parameter<double>("d_stop", 0.3);
    params_.d_slow = declare_parameter<double>("d_slow", 0.6);
    params_.cmd_timeout = declare_parameter<double>("cmd_timeout", 0.5);
    params_.scan_timeout = declare_parameter<double>("scan_timeout", 0.3);
    const double max_accel = declare_parameter<double>("max_accel", 1.0);
    params_.max_delta_v = max_accel / rate_hz;
    const auto cmd_qos_depth = declare_parameter<int>("cmd_qos_depth", 10);
    const auto scan_qos_depth = declare_parameter<int>("scan_qos_depth", 5);
    if (cmd_qos_depth < 1 || scan_qos_depth < 1) {
      throw std::invalid_argument("cmd_qos_depth and scan_qos_depth must be >= 1");
    }
    log_throttle_ms_ = declare_parameter<int>("log_throttle_ms", 2000);
    safety::validate(params_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(static_cast<std::size_t>(cmd_qos_depth)));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_nav", rclcpp::QoS(static_cast<std::size_t>(cmd_qos_depth)),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          cmd_.linear = msg->linear.x;
          cmd_.angular = msg->angular.z;
          cmd_time_ = steady_clock_.now();
          have_cmd_ = true;
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS().keep_last(static_cast<std::size_t>(scan_qos_depth)),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          scan_ = std::move(msg);
          scan_time_ = steady_clock_.now();
        });
    timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), [this]() { on_timer(); });

    RCLCPP_INFO(get_logger(),
                "rate %.1f Hz, cone %.1f deg (+%.1f deg per rad/s, max %.1f deg), d_stop %.2f m, "
                "d_slow %.2f m, timeouts cmd %.2f s / scan %.2f s, max_accel %.2f m/s^2",
                rate_hz, params_.cone_half_angle / kDegToRad, params_.cone_widen_gain / kDegToRad,
                params_.cone_max_half_angle / kDegToRad, params_.d_stop, params_.d_slow,
                params_.cmd_timeout, params_.scan_timeout, max_accel);
  }

 private:
  void on_timer() {
    const rclcpp::Time now = steady_clock_.now();
    const double inf = std::numeric_limits<double>::infinity();
    const double cmd_age = have_cmd_ ? (now - cmd_time_).seconds() : inf;
    const double scan_age = scan_ ? (now - scan_time_).seconds() : inf;

    safety::ScanView view;
    if (scan_) {
      view.angle_min = scan_->angle_min;
      view.angle_increment = scan_->angle_increment;
      view.range_min = scan_->range_min;
      view.range_max = scan_->range_max;
      view.ranges = &scan_->ranges;
    }
    const auto out = safety::gate_step(cmd_, last_linear_, cmd_age, scan_age, view, params_);
    last_linear_ = out.vel.linear;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = out.vel.linear;
    twist.angular.z = out.vel.angular;
    cmd_pub_->publish(twist);

    log_state(out, cmd_age, scan_age);
  }

  void log_state(const safety::GateOutput& out, double cmd_age, double scan_age) {
    if (out.state == last_state_) {
      return;
    }
    // Throttled so a flapping state cannot flood the log.
    auto& clk = *get_clock();
    switch (out.state) {
      case safety::GateState::kStaleCmd:
        // Normal when Nav2 is idle; debug level only.
        RCLCPP_DEBUG(get_logger(), "no /cmd_vel_nav for %.2f s -> zero", cmd_age);
        break;
      case safety::GateState::kStaleScan:
        RCLCPP_WARN_THROTTLE(get_logger(), clk, log_throttle_ms_,
                             "no /scan for %.2f s -> zero (watchdog)", scan_age);
        break;
      case safety::GateState::kInvalidCmd:
        RCLCPP_WARN_THROTTLE(get_logger(), clk, log_throttle_ms_,
                             "non-finite /cmd_vel_nav -> zero");
        break;
      case safety::GateState::kStopped:
        RCLCPP_WARN_THROTTLE(get_logger(), clk, log_throttle_ms_,
                             "obstacle within d_stop in the %s cone -> linear zero",
                             cmd_.linear < 0.0 ? "rear" : "forward");
        break;
      case safety::GateState::kScaled:
        RCLCPP_INFO_THROTTLE(get_logger(), clk, log_throttle_ms_, "slowing, scale %.2f", out.scale);
        break;
      case safety::GateState::kPass:
        RCLCPP_DEBUG(get_logger(), "clear");
        break;
    }
    last_state_ = out.state;
  }

  safety::GateParams params_;
  rclcpp::Clock steady_clock_;
  int log_throttle_ms_ = 0;

  // Written by the subscription callbacks, read by the timer. The node runs on a
  // single-threaded executor (main below), so callbacks and timer never run concurrently.
  safety::Velocity cmd_;
  rclcpp::Time cmd_time_{0, 0, RCL_STEADY_TIME};
  bool have_cmd_ = false;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  rclcpp::Time scan_time_{0, 0, RCL_STEADY_TIME};

  double last_linear_ = 0.0;
  safety::GateState last_state_ = safety::GateState::kStaleCmd;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace semnav_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<semnav_control::SafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
