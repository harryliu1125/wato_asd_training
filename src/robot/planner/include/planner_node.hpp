#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "planner_core.hpp"

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex & other) const
  {
    return x == other.x && y == other.y;
  }

  bool operator!=(const CellIndex & other) const
  {
    return x != other.x || y != other.y;
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex & idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
  bool operator()(const AStarNode & a, const AStarNode & b)
  {
    return a.f_score > b.f_score;
  }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State {
      WAITING_FOR_GOAL,
      WAITING_FOR_ROBOT_TO_REACH_GOAL
    };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    bool findNearestUnblockedCell(const CellIndex & input, CellIndex & output, int search_radius);

    void planPath();
    bool goalReached();

    bool worldToGrid(double world_x, double world_y, CellIndex & cell);
    geometry_msgs::msg::PoseStamped gridToPose(const CellIndex & cell);

    bool isCellInsideMap(const CellIndex & cell);
    bool isCellBlocked(const CellIndex & cell);
    int getCellCost(const CellIndex & cell);

    double heuristic(const CellIndex & a, const CellIndex & b);
    double movementCost(const CellIndex & from, const CellIndex & to);
    std::vector<CellIndex> getNeighbors(const CellIndex & cell);
    std::vector<CellIndex> reconstructPath(
      const std::unordered_map<CellIndex, CellIndex, CellIndexHash> & came_from,
      const CellIndex & current
    );

    std::vector<CellIndex> runAStar(const CellIndex & start, const CellIndex & goal);

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_;

    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    bool has_map_;
    bool has_odom_;
    bool goal_received_;

    double goal_tolerance_;
    int blocked_threshold_;
};

#endif