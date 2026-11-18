cmake_minimum_required(VERSION 3.8)
project(lsm_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lsm_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 1. C++17 5-Stage Logistics AMR Finite State Machine Node (Primary Production Node)
add_executable(amr_state_machine_node src/amr_state_machine_cpp_node.cpp)
ament_target_dependencies(amr_state_machine_node
  rclcpp
  rclcpp_action
  ament_index_cpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  lsm_interfaces
)
target_link_libraries(amr_state_machine_node yaml-cpp)

# 2. C++17 50Hz Closed-Loop PID Precision Docking & 360° LiDAR Safety Supervisor Node
add_executable(precision_docking_node src/precision_docking_node.cpp)
ament_target_dependencies(precision_docking_node
  rclcpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
)

# 3. C++17 206-Node Topological A* Route Planner & Eigen3 EKF Benchmark Node
add_executable(topological_route_planner_node src/topological_route_planner_node.cpp)
ament_target_dependencies(topological_route_planner_node
  rclcpp
  ament_index_cpp
  std_msgs
)
target_link_libraries(topological_route_planner_node yaml-cpp)

install(TARGETS
  amr_state_machine_node
  precision_docking_node
  topological_route_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install Python modules & legacy Python reference executables for Python vs C++ comparison
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  lsm_navigation/waypoint_navigator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_navigator
)

install(PROGRAMS
  lsm_navigation/amr_state_machine_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME amr_state_machine_py_node
)

install(PROGRAMS
  lsm_navigation/waypoint_navigator.py
  lsm_navigation/amr_state_machine_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch & Config & Maps & RViz directories
install(DIRECTORY
  config
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
