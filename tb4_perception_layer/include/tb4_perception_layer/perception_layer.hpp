#ifndef TB4_PERCEPTION_LAYER__PERCEPTION_LAYER_HPP_
#define TB4_PERCEPTION_LAYER__PERCEPTION_LAYER_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tb4_perception_layer/msg/semantic_obstacle_array.hpp"

namespace tb4_perception_layer
{

class PerceptionLayer : public nav2_costmap_2d::Layer
{
public:
  PerceptionLayer();

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j,
    int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void obstacleCallback(
    const tb4_perception_layer::msg::SemanticObstacleArray::SharedPtr msg);

  // Flush persistent_obstacles_ on demand (std_srvs/Trigger service).
  void clearPersistentCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  struct CachedObstacle
  {
    std::string class_id;
    std::string frame_id;
    double x;
    double y;
    double radius;
    unsigned char cost;
    rclcpp::Time stamp;
    // Persistent keep-outs only: latched true once the robot has come within
    // persistent_react_distance_. Prevents the reroute from oscillating as the
    // robot crosses the react boundary.
    bool activated{false};
  };

  unsigned char costForClass(const std::string & class_id) const;
  double radiusForClass(const std::string & class_id) const;
  bool isPersistentClass(const std::string & class_id) const;

  // Transform a 2-D point from one frame into another using tf_.
  // Returns false if the transform is unavailable.
  bool transformPoint(
    const std::string & from_frame, const std::string & to_frame,
    double in_x, double in_y, double & out_x, double & out_y) const;

  rclcpp::Subscription<msg::SemanticObstacleArray>::SharedPtr sub_;
  std::mutex mutex_;
  std::vector<CachedObstacle> obstacles_;
  // Latched keep-out zones (e.g. stop signs) stored in persistent_frame_.
  // These survive loss of sight and costmap clearing.
  std::vector<CachedObstacle> persistent_obstacles_;
  // Keep-outs just removed by the clear service, held one cycle so updateBounds
  // can expand over them and force the master grid to drop the stamped cost.
  std::vector<CachedObstacle> cleared_regions_;
  bool have_cleared_regions_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;

  // Parameters
  std::string semantic_topic_;
  double obstacle_timeout_;
  unsigned char default_cost_;
  double default_radius_;
  std::vector<std::string> persistent_classes_;
  std::string persistent_frame_;
  double persistent_timeout_;
  // Obstacles whose centre is closer than this (m) to the robot are ignored,
  // so a detection stamped on top of the robot cannot self-block navigation.
  double min_obstacle_distance_;
  // Persistent keep-outs (e.g. stop signs) only stamp cost once the robot is
  // within this distance (m). 0 = always stamp (disabled).
  double persistent_react_distance_{0.0};
  // After a costmap clear (e.g. Nav2 recovery calls reset()), transient obstacle
  // stamping is suppressed for this long so the live fusion stream cannot
  // instantly repopulate the same blob before the robot can move. Persistent
  // keep-outs are unaffected. 0 = disabled.
  double clear_suppression_time_{2.0};
  rclcpp::Time suppress_transient_until_;
  bool have_suppress_window_{false};
  std::unordered_map<std::string, unsigned char> class_cost_map_;
  std::unordered_map<std::string, double> class_radius_map_;

  // Latest robot position in the costmap's global frame, captured in
  // updateBounds() and reused in updateCosts() for the min-distance filter.
  double robot_gx_{0.0};
  double robot_gy_{0.0};
  bool have_robot_pose_{false};
};

}  // namespace tb4_perception_layer

#endif  // TB4_PERCEPTION_LAYER__PERCEPTION_LAYER_HPP_
