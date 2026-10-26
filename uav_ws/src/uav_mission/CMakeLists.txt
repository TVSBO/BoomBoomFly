cmake_minimum_required(VERSION 3.8)
project(uav_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uav_interfaces REQUIRED)

add_executable(mission_node
  src/flight_task.cpp
  src/mission_node.cpp
)
target_compile_features(mission_node PRIVATE cxx_std_17)
target_include_directories(mission_node PRIVATE include)
ament_target_dependencies(mission_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  std_srvs
  uav_interfaces
)

install(TARGETS mission_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_flight_task test/test_flight_task.cpp src/flight_task.cpp)
  target_include_directories(test_flight_task PRIVATE include)
  target_compile_features(test_flight_task PRIVATE cxx_std_17)
  ament_target_dependencies(test_flight_task geometry_msgs uav_interfaces)
endif()

ament_package()
