cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../config/openvins/d435i_px4
  DESTINATION share/${PROJECT_NAME}/config/openvins)
install(PROGRAMS scripts/check_vio.py scripts/record_vio.py scripts/analyze_vio_bag.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/vio_tools.py DESTINATION lib/${PROJECT_NAME})

# 所有启动场景统一使用 launch/uav.launch.py。

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(vio_tools_test test/test_vio_tools.py)
  ament_add_pytest_test(vio_bag_analyzer_test test/test_vio_bag_analyzer.py)
  ament_add_pytest_test(mission_launch_test test/test_mission_launch.py)
endif()

ament_package()
