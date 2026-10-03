// SemanticLayer: nav2_costmap_2d plugin that paints per-class costs around semantic obstacles
// (architecture 7.3). The cost math lives in cost_model.hpp (ROS-free, unit-tested).
// Deviation D-26: semantic cost is capped at max_semantic_cost (<= 252), never lethal; LiDAR (the
// obstacle layer) is the only lethal source.
#ifndef SEMNAV_COSTMAP__SEMANTIC_LAYER_HPP_
#define SEMNAV_COSTMAP__SEMANTIC_LAYER_HPP_

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semnav_costmap/cost_model.hpp"
#include "semnav_msgs/msg/semantic_obstacle_array.hpp"

namespace semnav_costmap {

class SemanticLayer : public nav2_costmap_2d::Layer {
 public:
  SemanticLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                    double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                   int max_j) override;
  void reset() override;
  // Semantic obstacles come from tracking, not raytracing: clearing services must not wipe them.
  bool isClearable() override { return false; }

 private:
  // One obstacle as received (frame of the message, expiry in ROS time nanoseconds).
  struct StoredObstacle {
    std::string class_name;
    std::string frame_id;
    double x = 0.0;
    double y = 0.0;
    std::int64_t expiry_ns = 0;
  };
  // One live obstacle in the costmap's global frame, ready to paint.
  struct LiveObstacle {
    double x = 0.0;
    double y = 0.0;
    ClassCostParams params;
  };
  struct WorldBounds {
    bool valid = false;
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
  };

  void declare_class_table(const std::string& table, const std::map<std::string, double>& defaults,
                           std::map<std::string, double>& out);
  void on_obstacles(const semnav_msgs::msg::SemanticObstacleArray::ConstSharedPtr msg);
  ClassCostParams params_for(const std::string& class_name) const;

  // Parameters (fixed after onInitialize).
  std::map<std::string, double> class_radius_;
  std::map<std::string, double> class_inflation_;
  double decay_k_ = 0.0;
  std::uint8_t max_semantic_cost_ = kDefaultMaxSemanticCost;  // 1..252, never lethal (D-26)
  double obstacle_timeout_ = 0.0;
  double log_throttle_period_ = 0.0;

  rclcpp::Subscription<semnav_msgs::msg::SemanticObstacleArray>::SharedPtr sub_;

  // Written by the subscription callback, copied at the start of updateBounds.
  std::mutex snapshot_mutex_;
  std::vector<StoredObstacle> snapshot_;

  // Costmap thread only.
  std::vector<LiveObstacle> live_;
  WorldBounds last_bounds_;  // area painted in the previous cycle, re-dirtied to clear it
};

}  // namespace semnav_costmap

#endif  // SEMNAV_COSTMAP__SEMANTIC_LAYER_HPP_
