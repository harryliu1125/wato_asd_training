# WATO ASD Training — ROS2 Autonomous Navigation Stack

A Dockerized ROS2 autonomous navigation project for a simulated robot. The robot uses lidar perception, occupancy-grid mapping, A* path planning, and Pure Pursuit control to navigate toward user-defined goal points in a Gazebo/Foxglove simulation environment.

video here: https://www.youtube.com/watch?v=21nqUwOupjU

## Overview

This project implements a full basic navigation pipeline:

```text
/lidar
  ↓
Costmap Node
  ↓ /costmap

/costmap + /odom/filtered
  ↓
Map Memory Node
  ↓ /map

/map + /goal_point + /odom/filtered
  ↓
Planner Node
  ↓ /path

/path + /odom/filtered
  ↓
Control Node
  ↓ /cmd_vel
```

## Features

- Converts lidar scans into a local inflated costmap
- Marks unknown, free, inflated, and occupied cells
- Builds a global map using odometry and local costmaps
- Uses A* path planning over an occupancy grid
- Publishes planned paths as `nav_msgs/msg/Path`
- Uses Pure Pursuit Control to follow generated paths
- Publishes velocity commands to `/cmd_vel`
- Visualizes the full stack in Foxglove

## Main ROS Topics

| Topic | Type | Purpose |
|---|---|---|
| `/lidar` | `sensor_msgs/msg/LaserScan` | Raw lidar scan data |
| `/costmap` | `nav_msgs/msg/OccupancyGrid` | Local obstacle and inflation map |
| `/odom/filtered` | `nav_msgs/msg/Odometry` | Robot position estimate |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Global remembered map |
| `/goal_point` | `geometry_msgs/msg/PointStamped` | User-defined navigation goal |
| `/path` | `nav_msgs/msg/Path` | A* planned route |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity command |

## Node Breakdown

### Costmap Node

The Costmap Node subscribes to `/lidar` and converts laser scan data into a local occupancy grid.

It:

- Starts cells as unknown (`-1`)
- Marks lidar-visible free space as `0`
- Marks obstacle hits as `100`
- Inflates obstacles to create safety buffers
- Publishes the result to `/costmap`

### Map Memory Node

The Map Memory Node combines local costmaps with odometry to build a global map.

It:

- Subscribes to `/costmap`
- Subscribes to `/odom/filtered`
- Uses robot position and orientation to transform local costmap cells into the global frame
- Publishes the remembered world map to `/map`

### Planner Node

The Planner Node uses A* to generate a path from the robot’s current position to a goal.

It:

- Subscribes to `/map`
- Subscribes to `/goal_point`
- Subscribes to `/odom/filtered`
- Converts world coordinates into grid cells
- Runs A* over the occupancy grid
- Avoids high-cost obstacle and inflation cells
- Publishes the planned route to `/path`

### Control Node

The Control Node uses Pure Pursuit Control to follow the path.

It:

- Subscribes to `/path`
- Subscribes to `/odom/filtered`
- Finds a lookahead point on the path
- Computes linear and angular velocity commands
- Publishes movement commands to `/cmd_vel`

## Running the Project

Build the Dockerized ROS2 workspace:

```bash
cd ~/wato_asd_training
./watod build
```

Start the simulation stack:

```bash
./watod up
```

Connect Foxglove to:

```text
ws://localhost:20000
```

Recommended Foxglove fixed frame:

```text
sim_world
```

Recommended Foxglove topics:

```text
/map
/costmap
/path
/odom/filtered
/cmd_vel
/tf
/tf_static
```

## Sending a Goal

Enter the robot container:

```bash
cd ~/wato_asd_training
./watod exec robot bash
source /opt/ros/humble/setup.bash
source /opt/watonomous/setup.bash
```

Publish a goal:

```bash
ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'sim_world'}, point: {x: 1.5, y: 0.0, z: 0.0}}"
```

Expected pipeline:

```text
/goal_point
→ /path
→ /cmd_vel
→ robot moves toward goal
```

## Technologies Used

- ROS2 Humble
- C++
- Docker / Docker Compose
- Gazebo / Ignition simulation
- Foxglove visualization
- Lidar-based perception
- Occupancy grid mapping
- A* path planning
- Pure Pursuit Control

## Project Structure

```text
src/robot/costmap
src/robot/map_memory
src/robot/planner
src/robot/control
```

## Notes

This project was completed as part of the WATonomous ASD training assignment. The goal was to implement a simplified autonomous navigation stack from perception to mapping, planning, and control.
