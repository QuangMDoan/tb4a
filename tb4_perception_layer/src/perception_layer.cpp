// Build and Run:  
//     # cd /home/qd/turtlebot4_ws 
//     # source /opt/ros/jazzy/setup.bash 
//     # colcon build --packages-select tb4_perception_layer 

//     # source /home/qd/turtlebot4_ws/install/setup.bash 
//     # colcon build --packages-select tb4_perception_integration

//     # source /home/qd/turtlebot4_ws/install/setup.bash 
//     # ros2 interface show tb4_perception_layer/msg/SemanticObstacle 
//     # ros2 interface show tb4_perception_layer/msg/SemanticObstacleArray

//     # source /home/qd/turtlebot4_ws/install/setup.bash 
//     # ros2 pkg executables tb4_perception_integration

#include "tb4_perception_layer/perception_layer.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"

PLUGINLIB_EXPORT_CLASS(tb4_perception_layer::PerceptionLayer, nav2_costmap_2d::Layer)

namespace tb4_perception_layer
{

PerceptionLayer::PerceptionLayer() {}

void PerceptionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PerceptionLayer: failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("semantic_topic", rclcpp::ParameterValue("/semantic_obstacles"));
  declareParameter("obstacle_timeout", rclcpp::ParameterValue(3.0));
  declareParameter("default_cost", rclcpp::ParameterValue(254));
  declareParameter("default_radius", rclcpp::ParameterValue(0.3));
  declareParameter("class_names", rclcpp::ParameterValue(std::vector<std::string>()));
  declareParameter("class_costs", rclcpp::ParameterValue(std::vector<int64_t>()));
  declareParameter("class_radii", rclcpp::ParameterValue(std::vector<double>()));
  declareParameter(
    "persistent_classes",
    rclcpp::ParameterValue(std::vector<std::string>({"stop sign"})));
  declareParameter("persistent_frame", rclcpp::ParameterValue("map"));
  declareParameter("persistent_timeout", rclcpp::ParameterValue(600.0));
  declareParameter("min_obstacle_distance", rclcpp::ParameterValue(0.35));
  declareParameter("persistent_react_distance", rclcpp::ParameterValue(0.0));
  declareParameter("clear_suppression_time", rclcpp::ParameterValue(2.0));

  bool enabled = true;
  node->get_parameter(name_ + ".enabled", enabled);
  enabled_ = enabled;

  node->get_parameter(name_ + ".semantic_topic", semantic_topic_);
  node->get_parameter(name_ + ".obstacle_timeout", obstacle_timeout_);
  node->get_parameter(name_ + ".persistent_classes", persistent_classes_);
  node->get_parameter(name_ + ".persistent_frame", persistent_frame_);
  node->get_parameter(name_ + ".persistent_timeout", persistent_timeout_);
  node->get_parameter(name_ + ".min_obstacle_distance", min_obstacle_distance_);
  node->get_parameter(name_ + ".persistent_react_distance", persistent_react_distance_);
  node->get_parameter(name_ + ".clear_suppression_time", clear_suppression_time_);

  int default_cost_int = 254;
  node->get_parameter(name_ + ".default_cost", default_cost_int);
  default_cost_ = static_cast<unsigned char>(
    std::clamp(default_cost_int, 0, 254));

  node->get_parameter(name_ + ".default_radius", default_radius_);

  std::vector<std::string> class_names;
  std::vector<int64_t> class_costs;
  std::vector<double> class_radii;
  node->get_parameter(name_ + ".class_names", class_names);
  node->get_parameter(name_ + ".class_costs", class_costs);
  node->get_parameter(name_ + ".class_radii", class_radii);

  size_t n = class_names.size();
  for (size_t i = 0; i < n; ++i) {
    if (i < class_costs.size()) {
      class_cost_map_[class_names[i]] = static_cast<unsigned char>(
        std::clamp(static_cast<int>(class_costs[i]), 0, 254));
    }
    if (i < class_radii.size()) {
      class_radius_map_[class_names[i]] = class_radii[i];
    }
  }

  sub_ = node->create_subscription<msg::SemanticObstacleArray>(
    semantic_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PerceptionLayer::obstacleCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "PerceptionLayer initialized — topic: %s, timeout: %.1fs, %zu class mappings",
    semantic_topic_.c_str(), obstacle_timeout_, n);

  current_ = true;
}

void PerceptionLayer::obstacleCallback(
  const msg::SemanticObstacleArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_.clear();

  // If a recent costmap clear opened a suppression window, keep transient
  // obstacles empty so the live fusion stream can't instantly re-stamp the
  // blob that a recovery just cleared. Persistent keep-outs still latch below.
  bool suppress_transient = false;
  if (have_suppress_window_) {
    auto node = node_.lock();
    if (node) {
      if (node->get_clock()->now() < suppress_transient_until_) {
        suppress_transient = true;
      } else {
        have_suppress_window_ = false;
      }
    }
  }

  const std::string msg_frame = msg->header.frame_id;

  for (const auto & obs : msg->obstacles) {
    CachedObstacle cached;
    cached.class_id = obs.class_id;
    cached.frame_id = obs.header.frame_id.empty() ? msg_frame : obs.header.frame_id;
    cached.x = obs.x;
    cached.y = obs.y;
    cached.radius = (obs.radius > 0.0) ? obs.radius : radiusForClass(obs.class_id);
    cached.cost = (obs.cost > 0.0)
      ? static_cast<unsigned char>(std::clamp(static_cast<int>(obs.cost), 0, 254))
      : costForClass(obs.class_id);
    cached.stamp = rclcpp::Time(obs.header.stamp);
    if (!suppress_transient) {
      obstacles_.push_back(cached);
    }

    if (!isPersistentClass(obs.class_id)) {continue;}

    // Latch the keep-out zone in a fixed frame (e.g. map) so it survives
    // loss of sight and costmap clearing.
    double px, py;
    if (!transformPoint(cached.frame_id, persistent_frame_, cached.x, cached.y, px, py)) {
      continue;  // transform unavailable this cycle; will latch on a later message
    }

    CachedObstacle keepout = cached;
    keepout.frame_id = persistent_frame_;
    keepout.x = px;
    keepout.y = py;

    // De-duplicate: merge with an existing latched zone of the same class
    // if it is within half a radius, otherwise append.
    bool merged = false;
    for (auto & existing : persistent_obstacles_) {
      if (existing.class_id != keepout.class_id) {continue;}
      if (std::hypot(existing.x - keepout.x, existing.y - keepout.y) <=
        0.5 * keepout.radius)
      {
        existing.x = keepout.x;
        existing.y = keepout.y;
        existing.radius = keepout.radius;
        existing.cost = keepout.cost;
        existing.stamp = keepout.stamp;
        merged = true;
        break;
      }
    }
    if (!merged) {
      persistent_obstacles_.push_back(keepout);
    }
  }
}

void PerceptionLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {return;}

  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();
  if (!node) {return;}

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  rclcpp::Time now = node->get_clock()->now();

  // Remember the robot pose (already in the costmap's global frame) so
  // updateCosts() can apply the same min-distance filter.
  robot_gx_ = robot_x;
  robot_gy_ = robot_y;
  have_robot_pose_ = true;

  auto expand = [&](const CachedObstacle & obs, double timeout, bool is_persistent) {
      double age = (now - obs.stamp).seconds();
      if (age > timeout) {return;}
      double wx, wy;
      if (!transformPoint(obs.frame_id, global_frame, obs.x, obs.y, wx, wy)) {return;}
      // Ignore detections stamped on top of the robot so they cannot
      // self-block navigation.
      if (std::hypot(wx - robot_x, wy - robot_y) < min_obstacle_distance_) {return;}
      // React-distance gate: a persistent keep-out (e.g. stop sign) only enters
      // the costmap once the robot is within persistent_react_distance_, so a
      // far-off sign does not reroute the robot early.
      if (is_persistent && persistent_react_distance_ > 0.0 &&
        std::hypot(wx - robot_x, wy - robot_y) > persistent_react_distance_) {return;}
      double r = obs.radius;
      *min_x = std::min(*min_x, wx - r);
      *min_y = std::min(*min_y, wy - r);
      *max_x = std::max(*max_x, wx + r);
      *max_y = std::max(*max_y, wy + r);
    };

  for (const auto & obs : obstacles_) {expand(obs, obstacle_timeout_, false);}
  for (const auto & obs : persistent_obstacles_) {expand(obs, persistent_timeout_, true);}
}

void PerceptionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  if (!enabled_) {return;}

  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();
  if (!node) {return;}

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  rclcpp::Time now = node->get_clock()->now();
  double resolution = master_grid.getResolution();

  auto stamp = [&](const CachedObstacle & obs, double timeout, bool is_persistent) {
      double age = (now - obs.stamp).seconds();
      if (age > timeout) {return;}

      // Transform the obstacle into the costmap's global frame.
      double wx, wy;
      if (!transformPoint(obs.frame_id, global_frame, obs.x, obs.y, wx, wy)) {return;}

      // Ignore detections stamped on top of the robot so they cannot
      // self-block navigation (mirrors the filter in updateBounds).
      if (have_robot_pose_ &&
        std::hypot(wx - robot_gx_, wy - robot_gy_) < min_obstacle_distance_)
      {
        return;
      }

      // React-distance gate for persistent keep-outs (mirrors updateBounds):
      // a stop sign only stamps cost once the robot is within react distance.
      if (is_persistent && persistent_react_distance_ > 0.0 && have_robot_pose_ &&
        std::hypot(wx - robot_gx_, wy - robot_gy_) > persistent_react_distance_)
      {
        return;
      }

      // Convert world position to map cell
      unsigned int mx, my;
      if (!master_grid.worldToMap(wx, wy, mx, my)) {
        return;  // obstacle is outside the costmap
      }

      double r = obs.radius;
      int cell_radius = static_cast<int>(std::ceil(r / resolution));

      int cx = static_cast<int>(mx);
      int cy = static_cast<int>(my);

      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          int px = cx + dx;
          int py = cy + dy;

          if (px < min_i || px >= max_i || py < min_j || py >= max_j) {
            continue;
          }

          // Check circular footprint
          double dist = std::hypot(dx * resolution, dy * resolution);
          if (dist > r) {continue;}

          unsigned char old_cost = master_grid.getCost(
            static_cast<unsigned int>(px), static_cast<unsigned int>(py));
          if (obs.cost > old_cost) {
            master_grid.setCost(
              static_cast<unsigned int>(px),
              static_cast<unsigned int>(py),
              obs.cost);
          }
        }
      }
    };

  for (const auto & obs : obstacles_) {stamp(obs, obstacle_timeout_, false);}
  for (const auto & obs : persistent_obstacles_) {stamp(obs, persistent_timeout_, true);}
}

void PerceptionLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_.clear();
  // A costmap clear (e.g. Nav2 recovery) resets this layer via reset(). Open a
  // suppression window so the live fusion stream cannot immediately repopulate
  // the transient blob before the robot has a chance to move out of it.
  auto node = node_.lock();
  if (node && clear_suppression_time_ > 0.0) {
    suppress_transient_until_ =
      node->get_clock()->now() + rclcpp::Duration::from_seconds(clear_suppression_time_);
    have_suppress_window_ = true;
  }
  // Note: persistent_obstacles_ are intentionally NOT cleared here so that
  // latched keep-out zones (e.g. stop signs) survive costmap resets/clears.
  current_ = false;
}

unsigned char PerceptionLayer::costForClass(const std::string & class_id) const
{
  auto it = class_cost_map_.find(class_id);
  return (it != class_cost_map_.end()) ? it->second : default_cost_;
}

double PerceptionLayer::radiusForClass(const std::string & class_id) const
{
  auto it = class_radius_map_.find(class_id);
  return (it != class_radius_map_.end()) ? it->second : default_radius_;
}

bool PerceptionLayer::isPersistentClass(const std::string & class_id) const
{
  return std::find(
    persistent_classes_.begin(), persistent_classes_.end(), class_id) !=
         persistent_classes_.end();
}

bool PerceptionLayer::transformPoint(
  const std::string & from_frame, const std::string & to_frame,
  double in_x, double in_y, double & out_x, double & out_y) const
{
  if (from_frame.empty() || from_frame == to_frame) {
    out_x = in_x;
    out_y = in_y;
    return true;
  }
  if (!tf_) {return false;}

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return false;
  }

  // Extract yaw directly from the quaternion (2-D transform only).
  const auto & q = tf.transform.rotation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  out_x = tf.transform.translation.x + c * in_x - s * in_y;
  out_y = tf.transform.translation.y + s * in_x + c * in_y;
  return true;
}

}  // namespace tb4_perception_layer
