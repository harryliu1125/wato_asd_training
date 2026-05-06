#include <cmath>
#include <memory>
#include <algorithm>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  map_memory_(robot::MapMemoryCore(this->get_logger())),
  has_costmap_(false),
  has_odom_(false),
  global_map_initialized_(false),
  first_update_(true),
  last_update_x_(0.0),
  last_update_y_(0.0),
  map_resolution_(0.2),
  map_width_(200),
  map_height_(200),
  map_origin_x_(-20.0),
  map_origin_y_(-20.0),
  update_distance_threshold_(0.9)
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map",
    10
  );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MapMemoryNode::timerCallback, this)
  );

  initializeGlobalMap();

  RCLCPP_INFO(
    this->get_logger(),
    "Map memory node started. Subscribing to /costmap and /odom/filtered, publishing /map."
  );
}

void MapMemoryNode::initializeGlobalMap()
{
  global_map_.header.stamp = this->get_clock()->now();
  global_map_.header.frame_id = "sim_world";

  global_map_.info.resolution = map_resolution_;
  global_map_.info.width = map_width_;
  global_map_.info.height = map_height_;

  global_map_.info.origin.position.x = map_origin_x_;
  global_map_.info.origin.position.y = map_origin_y_;
  global_map_.info.origin.position.z = 0.0;

  global_map_.info.origin.orientation.x = 0.0;
  global_map_.info.origin.orientation.y = 0.0;
  global_map_.info.origin.orientation.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  // -1 means unknown. This is useful for a global map because the robot
  // has not observed most of the world yet.
  global_map_.data.assign(map_width_ * map_height_, -1);

  global_map_initialized_ = true;
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  has_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
  has_odom_ = true;
}

double MapMemoryNode::distanceFromLastUpdate()
{
  double current_x = latest_odom_.pose.pose.position.x;
  double current_y = latest_odom_.pose.pose.position.y;

  double dx = current_x - last_update_x_;
  double dy = current_y - last_update_y_;

  return std::sqrt(dx * dx + dy * dy);
}

bool MapMemoryNode::worldToGlobalGrid(double world_x, double world_y, int & grid_x, int & grid_y)
{
  grid_x = static_cast<int>((world_x - map_origin_x_) / map_resolution_);
  grid_y = static_cast<int>((world_y - map_origin_y_) / map_resolution_);

  if (grid_x < 0 || grid_x >= map_width_ || grid_y < 0 || grid_y >= map_height_) {
    return false;
  }

  return true;
}

double MapMemoryNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  // Standard quaternion to yaw conversion
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  return std::atan2(siny_cosp, cosy_cosp);
}

void MapMemoryNode::integrateCostmap()
{
  if (!has_costmap_ || !has_odom_ || !global_map_initialized_) {
    return;
  }

  double robot_x = latest_odom_.pose.pose.position.x;
  double robot_y = latest_odom_.pose.pose.position.y;
  double robot_yaw = getYawFromQuaternion(latest_odom_.pose.pose.orientation);

  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);

  int local_width = static_cast<int>(latest_costmap_.info.width);
  int local_height = static_cast<int>(latest_costmap_.info.height);
  double local_resolution = latest_costmap_.info.resolution;
  double local_origin_x = latest_costmap_.info.origin.position.x;
  double local_origin_y = latest_costmap_.info.origin.position.y;

  for (int y = 0; y < local_height; ++y) {
    for (int x = 0; x < local_width; ++x) {
      int local_index = y * local_width + x;
      int8_t local_value = latest_costmap_.data[local_index];

      // Ignore unknown cells if any exist.
      if (local_value < 0) {
        continue;
      }

      // Convert local grid cell into local robot-frame meters.
      double local_x = local_origin_x + (static_cast<double>(x) + 0.5) * local_resolution;
      double local_y = local_origin_y + (static_cast<double>(y) + 0.5) * local_resolution;

      // Rotate + translate local costmap point into global/world frame.
      double world_x = robot_x + cos_yaw * local_x - sin_yaw * local_y;
      double world_y = robot_y + sin_yaw * local_x + cos_yaw * local_y;

      int global_x = 0;
      int global_y = 0;

      if (!worldToGlobalGrid(world_x, world_y, global_x, global_y)) {
        continue;
      }

      int global_index = global_y * map_width_ + global_x;

      // Fuse maps.
      // Unknown global cells get overwritten.
      // Known cells keep the highest cost so obstacles/danger zones are remembered.
      if (global_map_.data[global_index] < local_value) {
        global_map_.data[global_index] = local_value;
      }
    }
  }

  last_update_x_ = robot_x;
  last_update_y_ = robot_y;
  first_update_ = false;

  RCLCPP_INFO(this->get_logger(), "Integrated latest /costmap into /map.");
}

void MapMemoryNode::timerCallback()
{
  if (!global_map_initialized_) {
    initializeGlobalMap();
  }

  if (has_costmap_ && has_odom_) {
    if (first_update_ || distanceFromLastUpdate() >= update_distance_threshold_) {
      integrateCostmap();
    }
  }

  // Important: publish even if we did not integrate this tick.
  // The assignment says the map memory node should publish a map on initialization
  // so later planner nodes have a map available.
  global_map_.header.stamp = this->get_clock()->now();
  map_pub_->publish(global_map_);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    3000,
    "Published /map"
  );
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}