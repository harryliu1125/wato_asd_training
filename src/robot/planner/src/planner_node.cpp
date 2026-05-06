#include "planner_node.hpp"
#include <cmath>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

PlannerNode::PlannerNode()
: Node("planner"),
  planner_(robot::PlannerCore(this->get_logger())),
  state_(State::WAITING_FOR_GOAL),
  has_map_(false),
  has_odom_(false),
  goal_received_(false),
  goal_tolerance_(0.5),
  blocked_threshold_(65)
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point",
    10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/path",
    10
  );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&PlannerNode::timerCallback, this)
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Planner node started. Subscribing to /map, /goal_point, /odom/filtered and publishing /path."
  );
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  has_map_ = true;

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

  RCLCPP_INFO(
    this->get_logger(),
    "Received goal: x=%.2f, y=%.2f",
    goal_.point.x,
    goal_.point.y
  );

  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
  has_odom_ = true;
}

void PlannerNode::timerCallback()
{
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached. Waiting for next goal.");
    state_ = State::WAITING_FOR_GOAL;
    goal_received_ = false;

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->get_clock()->now();
    empty_path.header.frame_id = current_map_.header.frame_id.empty() ? "sim_world" : current_map_.header.frame_id;
    path_pub_->publish(empty_path);

    return;
  }

  planPath();
}

bool PlannerNode::goalReached()
{
  if (!goal_received_ || !has_odom_) {
    return false;
  }

  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;

  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

bool PlannerNode::worldToGrid(double world_x, double world_y, CellIndex & cell)
{
  if (current_map_.data.empty()) {
    return false;
  }

  double origin_x = current_map_.info.origin.position.x;
  double origin_y = current_map_.info.origin.position.y;
  double resolution = current_map_.info.resolution;

  cell.x = static_cast<int>((world_x - origin_x) / resolution);
  cell.y = static_cast<int>((world_y - origin_y) / resolution);

  return isCellInsideMap(cell);
}

geometry_msgs::msg::PoseStamped PlannerNode::gridToPose(const CellIndex & cell)
{
  geometry_msgs::msg::PoseStamped pose;

  pose.header.stamp = this->get_clock()->now();
  pose.header.frame_id = current_map_.header.frame_id.empty() ? "sim_world" : current_map_.header.frame_id;

  double origin_x = current_map_.info.origin.position.x;
  double origin_y = current_map_.info.origin.position.y;
  double resolution = current_map_.info.resolution;

  pose.pose.position.x = origin_x + (static_cast<double>(cell.x) + 0.5) * resolution;
  pose.pose.position.y = origin_y + (static_cast<double>(cell.y) + 0.5) * resolution;
  pose.pose.position.z = 0.0;

  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  return pose;
}

bool PlannerNode::isCellInsideMap(const CellIndex & cell)
{
  int width = static_cast<int>(current_map_.info.width);
  int height = static_cast<int>(current_map_.info.height);

  return cell.x >= 0 && cell.x < width && cell.y >= 0 && cell.y < height;
}

int PlannerNode::getCellCost(const CellIndex & cell)
{
  if (!isCellInsideMap(cell)) {
    return 100;
  }

  int width = static_cast<int>(current_map_.info.width);
  int index = cell.y * width + cell.x;

  if (index < 0 || index >= static_cast<int>(current_map_.data.size())) {
    return 100;
  }

  return static_cast<int>(current_map_.data[index]);
}

bool PlannerNode::isCellBlocked(const CellIndex & cell)
{
  // Treat the robot as having physical size, not just a single center point.
  // If map resolution is 0.2m, radius 4 cells = about 0.8m of clearance.
  const int robot_radius_cells = 2;

  for (int dy = -robot_radius_cells; dy <= robot_radius_cells; ++dy) {
    for (int dx = -robot_radius_cells; dx <= robot_radius_cells; ++dx) {
      CellIndex check_cell(cell.x + dx, cell.y + dy);

      if (!isCellInsideMap(check_cell)) {
        return true;
      }

      double distance_cells = std::sqrt(static_cast<double>(dx * dx + dy * dy));

      if (distance_cells > robot_radius_cells) {
        continue;
      }

      int cost = getCellCost(check_cell);

      // Unknown space is allowed because the map is incomplete.
      if (cost < 0) {
        continue;
      }

      // If any nearby cell is high-cost, this position is unsafe for the robot body.
      if (cost >= blocked_threshold_) {
        return true;
      }
    }
  }

  return false;
}

bool PlannerNode::findNearestUnblockedCell(
  const CellIndex & input,
  CellIndex & output,
  int search_radius
)
{
  if (isCellInsideMap(input) && !isCellBlocked(input)) {
    output = input;
    return true;
  }

  for (int radius = 1; radius <= search_radius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        CellIndex candidate(input.x + dx, input.y + dy);

        if (!isCellInsideMap(candidate)) {
          continue;
        }

        if (!isCellBlocked(candidate)) {
          output = candidate;

          RCLCPP_WARN(
            this->get_logger(),
            "Input cell was blocked. Snapped to nearby free cell: (%d, %d).",
            output.x,
            output.y
          );

          return true;
        }
      }
    }
  }

  return false;
}

double PlannerNode::heuristic(const CellIndex & a, const CellIndex & b)
{
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);

  return std::sqrt(dx * dx + dy * dy);
}

double PlannerNode::movementCost(const CellIndex & from, const CellIndex & to)
{
  double base_cost = heuristic(from, to);

  int map_cost = getCellCost(to);

  if (map_cost < 0) {
    return base_cost + 3.0;
  }

  return base_cost + static_cast<double>(map_cost) / 4.0;
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex & cell)
{
  std::vector<CellIndex> neighbors;

  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }

      CellIndex neighbor(cell.x + dx, cell.y + dy);

      if (!isCellInsideMap(neighbor)) {
        continue;
      }

      if (isCellBlocked(neighbor)) {
        continue;
      }

      neighbors.push_back(neighbor);
    }
  }

  return neighbors;
}

std::vector<CellIndex> PlannerNode::reconstructPath(
  const std::unordered_map<CellIndex, CellIndex, CellIndexHash> & came_from,
  const CellIndex & current
)
{
  std::vector<CellIndex> path;
  CellIndex current_cell = current;

  path.push_back(current_cell);

  while (came_from.find(current_cell) != came_from.end()) {
    current_cell = came_from.at(current_cell);
    path.push_back(current_cell);
  }

  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<CellIndex> PlannerNode::runAStar(const CellIndex & start, const CellIndex & goal)
{
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (closed_set.find(current) != closed_set.end()) {
      continue;
    }

    if (current == goal) {
      return reconstructPath(came_from, current);
    }

    closed_set.insert(current);

    for (const auto & neighbor : getNeighbors(current)) {
      if (closed_set.find(neighbor) != closed_set.end()) {
        continue;
      }

      double tentative_g_score = g_score[current] + movementCost(current, neighbor);

      if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;

        double f_score = tentative_g_score + heuristic(neighbor, goal);
        open_set.push(AStarNode(neighbor, f_score));
      }
    }
  }

  return {};
}

void PlannerNode::planPath()
{
  if (!has_map_ || !has_odom_ || !goal_received_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Cannot plan path yet: missing map, odometry, or goal."
    );
    return;
  }

  if (current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: map data is empty.");
    return;
  }

  CellIndex start;
  CellIndex goal;

  if (!worldToGrid(robot_pose_.position.x, robot_pose_.position.y, start)) {
    RCLCPP_WARN(this->get_logger(), "Robot position is outside the map.");
    return;
  }

  if (!worldToGrid(goal_.point.x, goal_.point.y, goal)) {
    RCLCPP_WARN(this->get_logger(), "Goal position is outside the map.");
    return;
  }

  if (isCellBlocked(goal)) {
  CellIndex adjusted_goal;

    if (!findNearestUnblockedCell(goal, adjusted_goal, 10)) {
      RCLCPP_WARN(this->get_logger(), "Goal cell is blocked and no nearby free cell was found.");
      return;
    }

    goal = adjusted_goal;
  }

  if (isCellBlocked(goal)) {
    RCLCPP_WARN(this->get_logger(), "Goal cell is blocked.");
    return;
  }

  std::vector<CellIndex> grid_path = runAStar(start, goal);

  if (grid_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "A* failed to find a path. Publishing empty path.");

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->get_clock()->now();
    empty_path.header.frame_id = current_map_.header.frame_id.empty() ? "sim_world" : current_map_.header.frame_id;
    path_pub_->publish(empty_path);

    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = current_map_.header.frame_id.empty() ? "sim_world" : current_map_.header.frame_id;

  // Optional path thinning: publish every few cells so the path is not insanely dense.
  const int stride = 2;

  for (size_t i = 0; i < grid_path.size(); i += stride) {
    path.poses.push_back(gridToPose(grid_path[i]));
  }

  // Ensure the final goal cell is included.
  if (!grid_path.empty()) {
    path.poses.push_back(gridToPose(grid_path.back()));
  }

  path_pub_->publish(path);

  RCLCPP_INFO(
    this->get_logger(),
    "Published /path with %zu poses.",
    path.poses.size()
  );
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}