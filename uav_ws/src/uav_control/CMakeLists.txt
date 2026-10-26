cmake_minimum_required(VERSION 3.8)
project(uav_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(uav_control_node src/main.cpp src/px4_dds_interface.cpp)
target_compile_features(uav_control_node PRIVATE cxx_std_17)
target_include_directories(uav_control_node PRIVATE include)
ament_target_dependencies(uav_control_node
  rclcpp ament_index_cpp geometry_msgs rcl_interfaces px4_msgs std_srvs uav_interfaces)

option(BUILD_MAVROS_INTERFACE "Build the MAVROS control backend" ON)
if(BUILD_MAVROS_INTERFACE)
  find_package(mavros_msgs REQUIRED)
  target_sources(uav_control_node PRIVATE src/mavros_interface.cpp)
  target_compile_definitions(uav_control_node PRIVATE BUILD_MAVROS_INTERFACE)
  ament_target_dependencies(uav_control_node mavros_msgs)
endif()

install(TARGETS uav_control_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(dds_readiness test/test_dds_readiness.py
    ENV "CONTROL_EXECUTABLE=$<TARGET_FILE:uav_control_node>" "ROS_DOMAIN_ID=83"
    TIMEOUT 90)
endif()

ament_package()
