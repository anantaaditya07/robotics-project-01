// semantic_fusion_node (architecture 7.2): YOLO bounding boxes + 2D LiDAR -> tracked semantic
// obstacles in the map frame.
//
// For each /detections message:
//   - fx, cx come from the cached /camera/camera_info (never parameters, D-05);
//   - the camera frame is the detection header frame_id, the laser frame is the scan header
//     frame_id (no hard-coded frame names, D-05);
//   - the scan closest in time to the detection stamp is taken from a ring buffer (max_scan_dt);
//   - TF camera -> laser and laser -> target_frame are looked up at the detection stamp; if TF
//     cannot resolve, the error is logged and the detection dropped (never a guessed transform);
//   - fusion.hpp gives the laser-frame position, tracker.hpp associates and smooths.
// Publishes /semantic_obstacles (semnav_msgs/SemanticObstacleArray, reliable depth 5, frame
// target_frame = map) and /semantic_markers (cylinder + text per obstacle, colour by class).
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <map>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <semnav_msgs/msg/semantic_obstacle_array.hpp>
#include <semnav_perception/fusion.hpp>
#include <semnav_perception/tracker.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace semnav_perception {

namespace {
constexpr char kRadiusPrefix[] = "class_radius.";
constexpr char kDefaultClass[] = "default";
constexpr int kFxIndex = 0;  // CameraInfo K, row-major 3x3
constexpr int kCxIndex = 2;
constexpr double kMarkerAlpha = 0.6;
constexpr double kTextOffset = 0.2;  // m above the cylinder top
constexpr double kTextHeight = 0.2;  // m
}  // namespace

class SemanticFusionNode : public rclcpp::Node {
 public:
  SemanticFusionNode() : rclcpp::Node("semantic_fusion_node") {
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    fusion_params_.sector_fraction = declare_parameter<double>("sector_fraction", 0.6);
    fusion_params_.min_range = declare_parameter<double>("min_range", 0.12);
    fusion_params_.max_range = declare_parameter<double>("max_range", 3.5);  // D-07
    fusion::validate(fusion_params_);
    TrackerParams tp;
    tp.assoc_gate = declare_parameter<double>("assoc_gate", 0.6);
    tp.ttl = declare_parameter<double>("ttl", 2.0);
    tp.smoothing_alpha = declare_parameter<double>("smoothing_alpha", 0.5);
    tracker_ = std::make_unique<Tracker>(tp);
    ttl_ = tp.ttl;
    scan_buffer_size_ = static_cast<std::size_t>(declare_parameter<int>("scan_buffer_size", 10));
    max_scan_dt_ = declare_parameter<double>("max_scan_dt", 0.1);  // 7.2 slop
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
    marker_height_ = declare_parameter<double>("marker_height", 1.0);
    const auto scan_qos_depth = declare_parameter<int>("scan_qos_depth", 5);
    const auto qos_depth = declare_parameter<int>("qos_depth", 5);
    const auto expiry_period = declare_parameter<double>("expiry_check_period", 0.5);
    declare_class_radii();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    obstacles_pub_ = create_publisher<semnav_msgs::msg::SemanticObstacleArray>(
        "semantic_obstacles", rclcpp::QoS(static_cast<std::size_t>(qos_depth)).reliable());
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("semantic_markers", 10);

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera/camera_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { on_camera_info(*msg); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS().keep_last(static_cast<std::size_t>(scan_qos_depth)),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(std::move(msg)); });
    det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        "detections", rclcpp::QoS(static_cast<std::size_t>(qos_depth)).reliable(),
        [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) { on_detections(*msg); });
    expiry_timer_ = create_wall_timer(std::chrono::duration<double>(expiry_period),
                                      [this]() { on_expiry_timer(); });

    std::string radii;
    for (const auto& [name, r] : class_radius_) {
      radii += (radii.empty() ? "" : ", ") + name + " " + std::to_string(r).substr(0, 4);
    }
    RCLCPP_INFO(get_logger(),
                "target_frame %s, sector_fraction %.2f, range [%.2f, %.2f] m, gate %.2f m, ttl "
                "%.1f s, alpha %.2f, class_radius {%s}",
                target_frame_.c_str(), fusion_params_.sector_fraction, fusion_params_.min_range,
                fusion_params_.max_range, tp.assoc_gate, tp.ttl, tp.smoothing_alpha, radii.c_str());
  }

 private:
  // class_radius.<name> parameters: defaults per 7.2, plus any extra class from overrides.
  void declare_class_radii() {
    const std::map<std::string, double> defaults{
        {"person", 0.35}, {"chair", 0.25}, {kDefaultClass, 0.3}};
    for (const auto& [name, r] : defaults) {
      class_radius_[name] = declare_parameter<double>(kRadiusPrefix + name, r);
    }
    for (const auto& [key, value] : get_node_parameters_interface()->get_parameter_overrides()) {
      if (key.rfind(kRadiusPrefix, 0) == 0 && !has_parameter(key)) {
        class_radius_[key.substr(sizeof(kRadiusPrefix) - 1)] =
            declare_parameter<double>(key, value.get<double>());
      }
    }
    for (const auto& [name, r] : class_radius_) {
      if (!(r >= 0.0)) {
        throw std::invalid_argument("class_radius." + name + " must be >= 0");
      }
    }
  }

  double radius_for(const std::string& cls) const {
    const auto it = class_radius_.find(cls);
    return it != class_radius_.end() ? it->second : class_radius_.at(kDefaultClass);
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo& msg) {
    const double fx = msg.k[kFxIndex];
    const double cx = msg.k[kCxIndex];
    if (!(fx > 0.0)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info has invalid fx %.3f",
                            fx);
      return;
    }
    if (!intrinsics_) {
      RCLCPP_INFO(get_logger(), "camera intrinsics cached: fx %.2f, cx %.2f (frame %s)", fx, cx,
                  msg.header.frame_id.c_str());
    }
    intrinsics_ = fusion::Intrinsics{fx, cx};
  }

  void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    scans_.push_back(std::move(msg));
    while (scans_.size() > scan_buffer_size_) {
      scans_.pop_front();
    }
  }

  sensor_msgs::msg::LaserScan::ConstSharedPtr closest_scan(const rclcpp::Time& stamp) const {
    sensor_msgs::msg::LaserScan::ConstSharedPtr best;
    double best_dt = max_scan_dt_;
    for (const auto& s : scans_) {
      const double dt = std::abs((rclcpp::Time(s->header.stamp) - stamp).seconds());
      if (dt <= best_dt) {
        best_dt = dt;
        best = s;
      }
    }
    return best;
  }

  static fusion::Transform to_fusion_transform(const geometry_msgs::msg::TransformStamped& t) {
    tf2::Quaternion q;
    tf2::fromMsg(t.transform.rotation, q);
    const tf2::Matrix3x3 m(q);
    fusion::Transform out{};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        out.rotation[static_cast<std::size_t>(3 * r + c)] = m[r][c];
      }
    }
    out.translation = {t.transform.translation.x, t.transform.translation.y,
                       t.transform.translation.z};
    return out;
  }

  void on_detections(const vision_msgs::msg::Detection2DArray& msg) {
    const rclcpp::Time stamp(msg.header.stamp);
    std::vector<Observation> observations;
    if (!msg.detections.empty()) {
      collect_observations(msg, stamp, observations);
    }
    const auto tracks = tracker_->update(observations, stamp.seconds());
    publish(tracks, msg.header.stamp);
  }

  void collect_observations(const vision_msgs::msg::Detection2DArray& msg,
                            const rclcpp::Time& stamp, std::vector<Observation>& out) {
    if (!intrinsics_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "no camera_info yet, dropping detections");
      return;
    }
    const auto scan = closest_scan(stamp);
    if (!scan) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "no scan within %.3f s of detection stamp %.3f, dropping detections",
                           max_scan_dt_, stamp.seconds());
      return;
    }
    const std::string& camera_frame = msg.header.frame_id;
    const std::string& laser_frame = scan->header.frame_id;
    geometry_msgs::msg::TransformStamped cam_to_laser;
    geometry_msgs::msg::TransformStamped laser_to_target;
    try {
      const auto timeout = tf2::durationFromSec(tf_timeout_);
      cam_to_laser = tf_buffer_->lookupTransform(laser_frame, camera_frame, stamp, timeout);
      laser_to_target = tf_buffer_->lookupTransform(target_frame_, laser_frame, stamp, timeout);
    } catch (const tf2::TransformException& e) {
      // D-05: fail loudly, never fall back to a guessed transform.
      RCLCPP_ERROR(get_logger(), "TF %s -> %s -> %s at %.3f failed, dropping %zu detection(s): %s",
                   camera_frame.c_str(), laser_frame.c_str(), target_frame_.c_str(),
                   stamp.seconds(), msg.detections.size(), e.what());
      return;
    }

    const fusion::Transform tf = to_fusion_transform(cam_to_laser);
    const fusion::ScanGeometry geom{scan->angle_min, scan->angle_increment, scan->range_min,
                                    scan->range_max};
    for (const auto& det : msg.detections) {
      if (det.results.empty()) {
        continue;
      }
      const auto& hyp = det.results.front().hypothesis;
      const double half_w = det.bbox.size_x / 2.0;
      const double u_min = det.bbox.center.position.x - half_w;
      const double u_max = det.bbox.center.position.x + half_w;
      const auto res = fusion::fuse(u_min, u_max, *intrinsics_, tf, geom, scan->ranges,
                                    radius_for(hyp.class_id), fusion_params_);
      if (!res.ok()) {
        RCLCPP_DEBUG(get_logger(), "%s [%.0f, %.0f] px: %s (%zu beams, %zu valid)",
                     hyp.class_id.c_str(), u_min, u_max, fusion::toString(res.status),
                     res.beams_in_sector, res.valid_beams);
        continue;
      }
      geometry_msgs::msg::PointStamped in;
      in.header.frame_id = laser_frame;
      in.header.stamp = msg.header.stamp;
      in.point.x = res.x;
      in.point.y = res.y;
      geometry_msgs::msg::PointStamped p;
      tf2::doTransform(in, p, laser_to_target);
      out.push_back(Observation{hyp.class_id, p.point.x, p.point.y, hyp.score, res.range});
    }
  }

  void on_expiry_timer() {
    // Expire ghosts even if /detections stops arriving.
    if (tracker_->tracks().empty()) {
      return;
    }
    const auto before = tracker_->tracks().size();
    tracker_->expire(now().seconds());
    if (tracker_->tracks().size() != before) {
      publish(tracker_->tracks(), now());
    }
  }

  static std::array<float, 3> class_colour(const std::string& cls) {
    // Stable per-class hue from the name: colour by class without a hand-written table.
    const double hue = static_cast<double>(std::hash<std::string>{}(cls) % 360U);
    const double h = hue / 60.0;
    const double x = 1.0 - std::abs(std::fmod(h, 2.0) - 1.0);
    const int sector = static_cast<int>(h);
    const std::array<std::array<double, 3>, 6> rgb{
        {{1, x, 0}, {x, 1, 0}, {0, 1, x}, {0, x, 1}, {x, 0, 1}, {1, 0, x}}};
    const auto& c = rgb[static_cast<std::size_t>(sector % 6)];
    return {static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2])};
  }

  void publish(const std::vector<Track>& tracks, const rclcpp::Time& stamp) {
    semnav_msgs::msg::SemanticObstacleArray arr;
    arr.header.frame_id = target_frame_;
    arr.header.stamp = stamp;
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = arr.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    const auto ttl = rclcpp::Duration::from_seconds(ttl_);

    for (const auto& t : tracks) {
      semnav_msgs::msg::SemanticObstacle o;
      o.header = arr.header;
      o.id = t.id;
      o.class_name = t.class_name;
      o.confidence = static_cast<float>(t.confidence);
      o.position.x = t.x;
      o.position.y = t.y;
      o.range = static_cast<float>(t.range);
      o.radius = static_cast<float>(radius_for(t.class_name));
      o.ttl = ttl;
      arr.obstacles.push_back(o);

      const auto colour = class_colour(t.class_name);
      visualization_msgs::msg::Marker cyl;
      cyl.header = arr.header;
      cyl.ns = "semantic_obstacles";
      cyl.id = static_cast<int>(t.id);
      cyl.type = visualization_msgs::msg::Marker::CYLINDER;
      cyl.action = visualization_msgs::msg::Marker::ADD;
      cyl.pose.position.x = t.x;
      cyl.pose.position.y = t.y;
      cyl.pose.position.z = marker_height_ / 2.0;
      cyl.pose.orientation.w = 1.0;
      cyl.scale.x = cyl.scale.y = 2.0 * o.radius;
      cyl.scale.z = marker_height_;
      cyl.color.r = colour[0];
      cyl.color.g = colour[1];
      cyl.color.b = colour[2];
      cyl.color.a = static_cast<float>(kMarkerAlpha);
      cyl.lifetime = ttl;
      markers.markers.push_back(cyl);

      visualization_msgs::msg::Marker text = cyl;
      text.ns = "semantic_labels";
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position.z = marker_height_ + kTextOffset;
      text.scale.x = text.scale.y = 0.0;
      text.scale.z = kTextHeight;
      text.color.r = text.color.g = text.color.b = text.color.a = 1.0F;
      char label[64];
      std::snprintf(label, sizeof(label), "%s %.2f #%u", t.class_name.c_str(), t.confidence, t.id);
      text.text = label;
      markers.markers.push_back(text);
    }
    obstacles_pub_->publish(arr);
    markers_pub_->publish(markers);
  }

  std::string target_frame_;
  fusion::Params fusion_params_{};
  std::unique_ptr<Tracker> tracker_;
  double ttl_;
  std::size_t scan_buffer_size_;
  double max_scan_dt_;
  double tf_timeout_;
  double marker_height_;
  std::map<std::string, double> class_radius_;

  std::optional<fusion::Intrinsics> intrinsics_;
  std::deque<sensor_msgs::msg::LaserScan::ConstSharedPtr> scans_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Publisher<semnav_msgs::msg::SemanticObstacleArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
};

}  // namespace semnav_perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<semnav_perception::SemanticFusionNode>());
  rclcpp::shutdown();
  return 0;
}
