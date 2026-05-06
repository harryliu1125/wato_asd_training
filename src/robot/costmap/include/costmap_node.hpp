#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    // Runs every time a new /lidar message arrives
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Costmap helper functions
    nav_msgs::msg::OccupancyGrid initializeCostmap();
    bool convertToGrid(double x, double y, int & x_grid, int & y_grid);
    void markFreeRay(nav_msgs::msg::OccupancyGrid & grid, double range, double angle);
    void markObstacle(nav_msgs::msg::OccupancyGrid & grid, int x_grid, int y_grid);
    void inflateObstacles(nav_msgs::msg::OccupancyGrid & grid);

    robot::CostmapCore costmap_;

    // ROS constructs
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // Local costmap parameters
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    double inflation_radius_;
};

#endif