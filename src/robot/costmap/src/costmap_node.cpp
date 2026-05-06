#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"),
  costmap_(robot::CostmapCore(this->get_logger())),
  resolution_(0.1),
  width_(200),
  height_(200),
  origin_x_(-10.0),
  origin_y_(-10.0),
  inflation_radius_(1.30)
{
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
  );

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Costmap node started. Subscribing to /lidar and publishing /costmap."
  );
}

nav_msgs::msg::OccupancyGrid CostmapNode::initializeCostmap()
{
  nav_msgs::msg::OccupancyGrid grid;

  grid.header.stamp = this->get_clock()->now();

  // This costmap is built directly from lidar data,
  // so we publish it in the lidar frame.
  grid.header.frame_id = "robot/chassis/lidar";

  grid.info.resolution = resolution_;
  grid.info.width = width_;
  grid.info.height = height_;

  grid.info.origin.position.x = origin_x_;
  grid.info.origin.position.y = origin_y_;
  grid.info.origin.position.z = 0.0;

  grid.info.origin.orientation.x = 0.0;
  grid.info.origin.orientation.y = 0.0;
  grid.info.origin.orientation.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // Important change:
  // -1 = unknown / not observed
  //  0 = observed free space
  // 100 = occupied obstacle
  //
  // Previously this was initialized to 0, which made the whole square look free.
  grid.data.assign(width_ * height_, -1);

  return grid;
}

bool CostmapNode::convertToGrid(double x, double y, int & x_grid, int & y_grid)
{
  x_grid = static_cast<int>((x - origin_x_) / resolution_);
  y_grid = static_cast<int>((y - origin_y_) / resolution_);

  if (x_grid < 0 || x_grid >= width_ || y_grid < 0 || y_grid >= height_) {
    return false;
  }

  return true;
}

void CostmapNode::markFreeRay(nav_msgs::msg::OccupancyGrid & grid, double range, double angle)
{
  // Walk along the lidar ray from the robot outward.
  // Every cell before the obstacle is observed free space.
  double step = resolution_;

  for (double r = 0.0; r < range; r += step) {
    double x = r * std::cos(angle);
    double y = r * std::sin(angle);

    int x_grid = 0;
    int y_grid = 0;

    if (!convertToGrid(x, y, x_grid, y_grid)) {
      continue;
    }

    int index = y_grid * width_ + x_grid;

    if (index >= 0 && index < static_cast<int>(grid.data.size())) {
      // Mark unknown cells as free.
      // Do not erase obstacles if something already got marked as occupied.
      if (grid.data[index] == -1) {
        grid.data[index] = 0;
      }
    }
  }
}

void CostmapNode::markObstacle(nav_msgs::msg::OccupancyGrid & grid, int x_grid, int y_grid)
{
  int index = y_grid * width_ + x_grid;

  if (index >= 0 && index < static_cast<int>(grid.data.size())) {
    grid.data[index] = 100;
  }
}

void CostmapNode::inflateObstacles(nav_msgs::msg::OccupancyGrid & grid)
{
  std::vector<int> obstacle_indices;

  // First collect only true obstacle cells.
  // We do this before inflation so inflated cells do not inflate other inflated cells.
  for (int i = 0; i < static_cast<int>(grid.data.size()); ++i) {
    if (grid.data[i] == 100) {
      obstacle_indices.push_back(i);
    }
  }

  int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

  for (int obstacle_index : obstacle_indices) {
    int obstacle_x = obstacle_index % width_;
    int obstacle_y = obstacle_index / width_;

    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        int new_x = obstacle_x + dx;
        int new_y = obstacle_y + dy;

        if (new_x < 0 || new_x >= width_ || new_y < 0 || new_y >= height_) {
          continue;
        }

        double distance = std::sqrt(
          static_cast<double>(dx * dx + dy * dy)
        ) * resolution_;

        if (distance > inflation_radius_) {
          continue;
        }

        int index = new_y * width_ + new_x;

        int inflated_cost = static_cast<int>(
          100.0 * std::pow(1.0 - distance / inflation_radius_, 0.7)
        );

        inflated_cost = std::clamp(inflated_cost, 0, 100);

        // If this cell was unknown (-1), inflation makes it a known risky cell.
        // If this cell was free, inflation raises its cost.
        // If it was already a higher cost, keep the higher cost.
        if (inflated_cost > grid.data[index]) {
          grid.data[index] = inflated_cost;
        }
      }
    }
  }
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  auto grid = initializeCostmap();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double raw_range = scan->ranges[i];
    double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;

    // Case 1:
    // Valid lidar hit. Mark free space up to the hit, then mark obstacle.
    if (std::isfinite(raw_range) &&
        raw_range >= scan->range_min &&
        raw_range <= scan->range_max)
    {
      markFreeRay(grid, raw_range, angle);

      double x = raw_range * std::cos(angle);
      double y = raw_range * std::sin(angle);

      int x_grid = 0;
      int y_grid = 0;

      if (convertToGrid(x, y, x_grid, y_grid)) {
        markObstacle(grid, x_grid, y_grid);
      }
    }

    // Case 2:
    // No obstacle detected within max range.
    // Still mark the ray as observed free space out to max lidar range.
    else if (!std::isfinite(raw_range) || raw_range > scan->range_max)
    {
      markFreeRay(grid, scan->range_max, angle);
    }

    // Case 3:
    // Too close / invalid low reading. Ignore it.
    else {
      continue;
    }
  }

  inflateObstacles(grid);

  grid.header.stamp = this->get_clock()->now();
  costmap_pub_->publish(grid);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Published /costmap"
  );
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}