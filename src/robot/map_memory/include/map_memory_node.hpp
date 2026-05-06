#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    void initializeGlobalMap();
    void integrateCostmap();
    bool worldToGlobalGrid(double world_x, double world_y, int & grid_x, int & grid_y);
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
    double distanceFromLastUpdate();

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::Odometry latest_odom_;

    bool has_costmap_;
    bool has_odom_;
    bool global_map_initialized_;
    bool first_update_;

    double last_update_x_;
    double last_update_y_;

    double map_resolution_;
    int map_width_;
    int map_height_;
    double map_origin_x_;
    double map_origin_y_;
    double update_distance_threshold_;
};

#endif