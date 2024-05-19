cmake_minimum_required(VERSION 3.8)
project(capstone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
include_directories(include)

add_executable(map_server src/Map_Server.cpp)
ament_target_dependencies(map_server rclcpp std_msgs nav_msgs sensor_msgs)

add_executable(local_1 src/Turtle_1_Localization.cpp include/particle_filter/particle_filter.cpp)
ament_target_dependencies(local_1 rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

add_executable(local_2 src/Turtle_2_Localization.cpp include/particle_filter/particle_filter.cpp)
ament_target_dependencies(local_2 rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

add_executable(local_3 src/Turtle_3_Localization.cpp include/particle_filter/particle_filter.cpp)
ament_target_dependencies(local_3 rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

add_executable(local_4 src/Turtle_4_Localization.cpp include/particle_filter/particle_filter.cpp)
ament_target_dependencies(local_4 rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

add_executable(planning_1 src/Turtle_1_Planning.cpp include/particle_filter/particle_filter.cpp include/astar/AStar.cpp)
ament_target_dependencies(planning_1 rclcpp std_msgs nav_msgs)


# add_executable(astar src/astar_example.cpp include/astar/AStar.cpp)
# ament_target_dependencies(astar)


install(TARGETS
  local_1
  local_2
  local_3
  local_4
  map_server
  planning_1
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/Turtle_1_Control.py
  scripts/center_node.py
  scripts/Map_Plot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
