// SemanticLayer implementation (architecture 7.3).
#include "semnav_costmap/semantic_layer.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace semnav_costmap {

namespace {
constexpr char kDefaultClass[] = "default";
constexpr char kRadiusTable[] = "class_radius";
constexpr char kInflationTable[] = "class_inflation";
constexpr double kMsPerSecond = 1000.0;
}  // namespace

void SemanticLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SemanticLayer: failed to lock the costmap node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/semantic_obstacles")));
  declareParameter("qos_depth", rclcpp::ParameterValue(5));
  declareParameter("decay_k", rclcpp::ParameterValue(3.0));
  // D-26: semantic cost is never lethal; LiDAR is the only lethal source.
  declareParameter("max_semantic_cost",
                   rclcpp::ParameterValue(static_cast<int>(kDefaultMaxSemanticCost)));
  declareParameter("obstacle_timeout", rclcpp::ParameterValue(2.0));
  declareParameter("log_throttle_period", rclcpp::ParameterValue(5.0));

  std::string topic;
  int qos_depth = 0;
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("topic"), topic);
  node->get_parameter(getFullName("qos_depth"), qos_depth);
  node->get_parameter(getFullName("decay_k"), decay_k_);
  int max_semantic_cost = 0;
  node->get_parameter(getFullName("max_semantic_cost"), max_semantic_cost);
  node->get_parameter(getFullName("obstacle_timeout"), obstacle_timeout_);
  node->get_parameter(getFullName("log_throttle_period"), log_throttle_period_);

  // Class tables: defaults per 7.2 (radius, must match perception_params.yaml) and 7.3
  // (inflation), plus any extra class given as an override.
  declare_class_table(kRadiusTable, {{"person", 0.35}, {"chair", 0.25}, {kDefaultClass, 0.3}},
                      class_radius_);
  declare_class_table(kInflationTable, {{"person", 1.0}, {"chair", 0.3}, {kDefaultClass, 0.2}},
                      class_inflation_);

  // 1..252 so the semantic cost can never reach INSCRIBED (253) or LETHAL (254).
  if (!valid_max_semantic_cost(max_semantic_cost)) {
    throw std::invalid_argument("SemanticLayer " + name_ + ": max_semantic_cost must be in [1, " +
                                std::to_string(kMaxNonObstacle) + "] (got " +
                                std::to_string(max_semantic_cost) +
                                "); semantic cost must never be inscribed (253) or lethal (254)");
  }
  max_semantic_cost_ = static_cast<std::uint8_t>(max_semantic_cost);

  if (!(decay_k_ >= 0.0) || !(obstacle_timeout_ > 0.0) || qos_depth < 1 ||
      !(log_throttle_period_ >= 0.0)) {
    throw std::invalid_argument(
        "SemanticLayer " + name_ +
        ": need decay_k >= 0, obstacle_timeout > 0, qos_depth >= 1, log_throttle_period >= 0");
  }

  // A disabled layer stays inert: no subscription, nothing painted (`enabled` is read once here).
  if (enabled_) {
    // Callback group spun by the costmap's dedicated executor thread.
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    sub_ = node->create_subscription<semnav_msgs::msg::SemanticObstacleArray>(
        topic, rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(qos_depth))).reliable(),
        [this](const semnav_msgs::msg::SemanticObstacleArray::ConstSharedPtr msg) {
          on_obstacles(msg);
        },
        options);
  }

  current_ = true;

  std::string table;
  for (const auto& [cls, r] : class_radius_) {
    const auto p = params_for(cls);
    table += cls + " r=" + std::to_string(r) + " infl=" + std::to_string(p.inflation) + "; ";
  }
  for (const auto& [cls, infl] : class_inflation_) {
    if (class_radius_.count(cls) == 0) {
      table += cls + " r=default infl=" + std::to_string(infl) + "; ";
    }
  }
  RCLCPP_INFO(logger_,
              "SemanticLayer %s: %s on %s, k %.2f, max cost %d (non-lethal), timeout %.1f s, "
              "classes {%s}",
              name_.c_str(), enabled_ ? "enabled" : "disabled", topic.c_str(), decay_k_,
              static_cast<int>(max_semantic_cost_), obstacle_timeout_, table.c_str());
}

void SemanticLayer::declare_class_table(const std::string& table,
                                        const std::map<std::string, double>& defaults,
                                        std::map<std::string, double>& out) {
  auto node = node_.lock();
  for (const auto& [cls, value] : defaults) {
    declareParameter(table + "." + cls, rclcpp::ParameterValue(value));
  }
  const std::string prefix = getFullName(table + ".");
  for (const auto& [key, value] :
       node->get_node_parameters_interface()->get_parameter_overrides()) {
    if (key.rfind(prefix, 0) == 0 && !node->has_parameter(key)) {
      declareParameter(key.substr(name_.size() + 1), value);
    }
  }
  std::map<std::string, double> values;
  node->get_parameters(getFullName(table), values);
  for (const auto& [key, v] : values) {
    if (!(v >= 0.0)) {
      throw std::invalid_argument("SemanticLayer " + name_ + ": " + table + "." + key +
                                  " must be >= 0");
    }
    out[key] = v;
  }
  if (out.count(kDefaultClass) == 0) {
    throw std::invalid_argument("SemanticLayer " + name_ + ": " + table + ".default missing");
  }
}

ClassCostParams SemanticLayer::params_for(const std::string& class_name) const {
  const auto lookup = [&class_name](const std::map<std::string, double>& m) {
    const auto it = m.find(class_name);
    return it != m.end() ? it->second : m.at(kDefaultClass);
  };
  return ClassCostParams{lookup(class_radius_), lookup(class_inflation_), decay_k_,
                         max_semantic_cost_};
}

void SemanticLayer::on_obstacles(
    const semnav_msgs::msg::SemanticObstacleArray::ConstSharedPtr msg) {
  // Each array is the full set of live tracks from the fusion node: it replaces the snapshot.
  std::vector<StoredObstacle> next;
  next.reserve(msg->obstacles.size());
  const std::int64_t timeout_ns = rclcpp::Duration::from_seconds(obstacle_timeout_).nanoseconds();
  for (const auto& o : msg->obstacles) {
    // Expiry = header stamp + ttl. Fallbacks: array stamp if the obstacle has none, receive time
    // if neither has one; obstacle_timeout if ttl is zero.
    std::int64_t stamp_ns = rclcpp::Time(o.header.stamp).nanoseconds();
    if (stamp_ns == 0) {
      stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    }
    if (stamp_ns == 0) {
      stamp_ns = clock_->now().nanoseconds();
    }
    std::int64_t ttl_ns = rclcpp::Duration(o.ttl).nanoseconds();
    if (ttl_ns <= 0) {
      ttl_ns = timeout_ns;
    }
    next.push_back(StoredObstacle{
        o.class_name, o.header.frame_id.empty() ? msg->header.frame_id : o.header.frame_id,
        o.position.x, o.position.y, stamp_ns + ttl_ns});
  }
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_ = std::move(next);
}

void SemanticLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                 double* min_x, double* min_y, double* max_x, double* max_y) {
  if (!enabled_) {
    return;
  }
  std::vector<StoredObstacle> snapshot;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot = snapshot_;
  }

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  const std::int64_t now_ns = clock_->now().nanoseconds();
  const auto throttle_ms = static_cast<std::int64_t>(log_throttle_period_ * kMsPerSecond);
  std::map<std::string, geometry_msgs::msg::TransformStamped> transforms;  // per source frame

  live_.clear();
  WorldBounds bounds;
  const auto touch = [&bounds](double x0, double y0, double x1, double y1) {
    if (!bounds.valid) {
      bounds = WorldBounds{true, x0, y0, x1, y1};
      return;
    }
    bounds.min_x = std::min(bounds.min_x, x0);
    bounds.min_y = std::min(bounds.min_y, y0);
    bounds.max_x = std::max(bounds.max_x, x1);
    bounds.max_y = std::max(bounds.max_y, y1);
  };

  for (const auto& o : snapshot) {
    if (o.expiry_ns < now_ns) {
      continue;
    }
    double x = o.x;
    double y = o.y;
    if (o.frame_id != global_frame) {
      auto it = transforms.find(o.frame_id);
      if (it == transforms.end()) {
        try {
          // Latest available transform, non-blocking (no timeout on the costmap thread).
          it = transforms
                   .emplace(o.frame_id,
                            tf_->lookupTransform(global_frame, o.frame_id, tf2::TimePointZero))
                   .first;
        } catch (const tf2::TransformException& ex) {
          RCLCPP_ERROR_THROTTLE(logger_, *clock_, throttle_ms,
                                "SemanticLayer %s: cannot transform obstacle from '%s' to '%s': %s",
                                name_.c_str(), o.frame_id.c_str(), global_frame.c_str(), ex.what());
          continue;
        }
      }
      geometry_msgs::msg::PointStamped in;
      geometry_msgs::msg::PointStamped out;
      in.point.x = o.x;
      in.point.y = o.y;
      tf2::doTransform(in, out, it->second);
      x = out.point.x;
      y = out.point.y;
    }
    const ClassCostParams p = params_for(o.class_name);
    const double reach = p.radius + p.inflation;
    touch(x - reach, y - reach, x + reach, y + reach);
    live_.push_back(LiveObstacle{x, y, p});
  }

  // Re-dirty last cycle's painted area too, so a vanished obstacle is cleared from the master.
  const WorldBounds painted = bounds;
  if (last_bounds_.valid) {
    touch(last_bounds_.min_x, last_bounds_.min_y, last_bounds_.max_x, last_bounds_.max_y);
  }
  last_bounds_ = painted;

  if (bounds.valid) {
    *min_x = std::min(*min_x, bounds.min_x);
    *min_y = std::min(*min_y, bounds.min_y);
    *max_x = std::max(*max_x, bounds.max_x);
    *max_y = std::max(*max_y, bounds.max_y);
  }
}

void SemanticLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                int max_i, int max_j) {
  if (!enabled_ || live_.empty()) {
    return;
  }
  GridView grid;
  grid.data = master_grid.getCharMap();
  grid.width = static_cast<int>(master_grid.getSizeInCellsX());
  grid.height = static_cast<int>(master_grid.getSizeInCellsY());
  grid.resolution = master_grid.getResolution();
  grid.origin_x = master_grid.getOriginX();
  grid.origin_y = master_grid.getOriginY();
  const CellWindow clip{min_i, min_j, max_i, max_j};
  for (const auto& o : live_) {
    paint_disc(grid, o.x, o.y, o.params, clip);
  }
}

void SemanticLayer::reset() {
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.clear();
  }
  live_.clear();
  last_bounds_ = WorldBounds{};
  current_ = true;
}

}  // namespace semnav_costmap

PLUGINLIB_EXPORT_CLASS(semnav_costmap::SemanticLayer, nav2_costmap_2d::Layer)
