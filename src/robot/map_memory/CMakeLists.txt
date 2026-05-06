cmake_minimum_required(VERSION 3.10)
project(map_memory)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Compiles source files into a library
add_library(map_memory_lib
  src/map_memory_core.cpp
)

target_include_directories(map_memory_lib
  PUBLIC include
)

ament_target_dependencies(map_memory_lib
  rclcpp
  nav_msgs
  geometry_msgs
)

# Create ROS2 node executable from source files
add_executable(map_memory_node
  src/map_memory_node.cpp
)

target_link_libraries(map_memory_node
  map_memory_lib
)

ament_target_dependencies(map_memory_node
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  map_memory_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()