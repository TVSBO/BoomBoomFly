cmake_minimum_required(VERSION 3.8)
project(uav_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(imu_bridge src/imu_bridge.cpp)
target_compile_features(imu_bridge PUBLIC cxx_std_17)
ament_target_dependencies(imu_bridge rcl_interfaces rclcpp sensor_msgs px4_msgs)

add_executable(odom_bridge src/odom_bridge.cpp)
target_compile_features(odom_bridge PUBLIC cxx_std_17)
target_include_directories(odom_bridge PRIVATE include)
target_link_libraries(odom_bridge Eigen3::Eigen)
ament_target_dependencies(
  odom_bridge builtin_interfaces diagnostic_msgs geometry_msgs nav_msgs px4_msgs
  rcl_interfaces rclcpp tf2_ros)

install(TARGETS imu_bridge odom_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_vio_safety_gate test/test_vio_safety_gate.cpp)
  target_compile_features(test_vio_safety_gate PUBLIC cxx_std_17)
  target_include_directories(test_vio_safety_gate PRIVATE include)
  target_link_libraries(test_vio_safety_gate Eigen3::Eigen)
  add_test(NAME test_vio_safety_gate COMMAND test_vio_safety_gate)
endif()

ament_package()
