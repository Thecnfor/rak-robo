cmake_minimum_required(VERSION 3.8)
project(drone_navigation_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

add_library(flight_core src/flight_core.cpp)
target_include_directories(flight_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(px4_state_adapter src/px4_state_adapter_node.cpp)
ament_target_dependencies(px4_state_adapter nav_msgs px4_msgs rclcpp std_msgs)
target_link_libraries(px4_state_adapter flight_core)

add_executable(ego_local_planner src/ego_local_planner_node.cpp)
ament_target_dependencies(
  ego_local_planner geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs
  tf2 tf2_geometry_msgs tf2_ros tf2_sensor_msgs)
target_link_libraries(ego_local_planner flight_core)

add_executable(trajectory_executor src/trajectory_executor_node.cpp)
ament_target_dependencies(
  trajectory_executor geometry_msgs nav_msgs px4_msgs rclcpp std_msgs)
target_link_libraries(trajectory_executor flight_core)

add_executable(flight_supervisor src/flight_supervisor_node.cpp)
ament_target_dependencies(
  flight_supervisor geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs)
target_link_libraries(flight_supervisor flight_core)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ego_local_planner "${cpp_typesupport_target}")
target_link_libraries(trajectory_executor "${cpp_typesupport_target}")

install(TARGETS flight_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS
  px4_state_adapter ego_local_planner trajectory_executor flight_supervisor
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_flight_core test/test_flight_core.cpp)
  target_link_libraries(test_flight_core flight_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(flight_core)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
