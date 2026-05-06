cmake_minimum_required(VERSION 3.10)
project(costmap)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Compiles source files into a library
add_library(costmap_lib
  src/costmap_core.cpp
)

# Indicate to compiler where to search for header files
target_include_directories(costmap_lib
  PUBLIC include
)

# Add ROS2 dependencies required by library
ament_target_dependencies(costmap_lib
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
)

# Create ROS2 node executable from source files
add_executable(costmap_node
  src/costmap_node.cpp
)

# Link to the previously built library to access costmap classes and methods
target_link_libraries(costmap_node
  costmap_lib
)

# Add ROS2 dependencies required by node executable
ament_target_dependencies(costmap_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
)

# Copy executable to installation location
install(TARGETS
  costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

# Copy launch and config files to installation location
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()