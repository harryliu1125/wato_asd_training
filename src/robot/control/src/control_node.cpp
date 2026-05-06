#include "control_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>

ControlNode::ControlNode()
: Node("control"),
control_(robot::ControlCore(this->get_logger())),
lookahead_distance_(0.55),
goal_tolerance_(0.30),
linear_speed_(0.6),
angular_gain_(2.8),
max_angular_speed_(3.0)
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path",
    10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  );

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10
  );

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&ControlNode::controlLoop, this)
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Control node started. Subscribing to /path and /odom/filtered, publishing /cmd_vel."
  );
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = msg;

  RCLCPP_INFO(
    this->get_logger(),
    "Received /path with %zu poses.",
    current_path_->poses.size()
  );
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
}

double ControlNode::computeDistance(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b
)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;

  return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion & q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  return std::atan2(siny_cosp, cosy_cosp);
}

double ControlNode::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint()
{
  if (!current_path_ || !current_odom_) {
    return std::nullopt;
  }

  if (current_path_->poses.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point robot_position = current_odom_->pose.pose.position;

  // Step 1: find the closest point on the path to the robot.
  size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < current_path_->poses.size(); ++i) {
    double distance = computeDistance(robot_position, current_path_->poses[i].pose.position);

    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  // Step 2: starting from the closest point, look forward along the path.
  for (size_t i = closest_index; i < current_path_->poses.size(); ++i) {
    double distance = computeDistance(robot_position, current_path_->poses[i].pose.position);

    if (distance >= lookahead_distance_) {
      return current_path_->poses[i];
    }
  }

  // Step 3: if no point is far enough, use the final goal.
  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocityCommand(
  const geometry_msgs::msg::PoseStamped & target
)
{
  geometry_msgs::msg::Twist cmd_vel;

  geometry_msgs::msg::Point robot_position = current_odom_->pose.pose.position;
  double robot_yaw = extractYaw(current_odom_->pose.pose.orientation);

  double dx = target.pose.position.x - robot_position.x;
  double dy = target.pose.position.y - robot_position.y;

  double target_heading = std::atan2(dy, dx);
  double heading_error = normalizeAngle(target_heading - robot_yaw);

  double distance_to_target = std::sqrt(dx * dx + dy * dy);

  // If the target is mostly not in front of us, rotate first instead of driving into walls.
  if (std::abs(heading_error) > 0.75) {
    cmd_vel.linear.x = 0.15;
    cmd_vel.angular.z = std::clamp(
      angular_gain_ * heading_error,
      -max_angular_speed_,
      max_angular_speed_
    );

    return cmd_vel;
  }

  // Slow down when turning.
  double turn_slowdown = 1.0 - std::min(std::abs(heading_error), 0.75) / 0.75;
  double speed = linear_speed_ * std::clamp(turn_slowdown, 0.25, 1.0);

  // Slow down near the lookahead point.
  if (distance_to_target < lookahead_distance_) {
    speed *= distance_to_target / lookahead_distance_;
  }

  speed = std::clamp(speed, 0.08, linear_speed_);

  cmd_vel.linear.x = speed;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;

  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = std::clamp(
    angular_gain_ * heading_error,
    -max_angular_speed_,
    max_angular_speed_
  );

  return cmd_vel;
}

void ControlNode::publishStopCommand()
{
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.linear.y = 0.0;
  stop_cmd.linear.z = 0.0;
  stop_cmd.angular.x = 0.0;
  stop_cmd.angular.y = 0.0;
  stop_cmd.angular.z = 0.0;

  cmd_vel_pub_->publish(stop_cmd);
}

void ControlNode::controlLoop()
{
  if (!current_path_ || !current_odom_) {
    return;
  }

  if (current_path_->poses.empty()) {
    publishStopCommand();
    return;
  }

  geometry_msgs::msg::Point robot_position = current_odom_->pose.pose.position;
  geometry_msgs::msg::Point final_goal = current_path_->poses.back().pose.position;

  double distance_to_goal = computeDistance(robot_position, final_goal);

  if (distance_to_goal < goal_tolerance_) {
    publishStopCommand();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Reached final path goal. Stopping."
    );

    return;
  }

  auto lookahead_point = findLookaheadPoint();

  if (!lookahead_point) {
    publishStopCommand();
    return;
  }

  auto cmd_vel = computeVelocityCommand(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Publishing /cmd_vel: linear=%.2f angular=%.2f",
    cmd_vel.linear.x,
    cmd_vel.angular.z
  );
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}