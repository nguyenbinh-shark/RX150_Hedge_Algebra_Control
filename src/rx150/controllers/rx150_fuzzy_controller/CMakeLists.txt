cmake_minimum_required(VERSION 3.5)
project(rx150_fuzzy_controller)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
# TOTG online (profile vận tốc/gia tốc trước bộ fuzzy). apt: ros-humble-ruckig.
find_package(ruckig CONFIG REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)
# Engine Mamdani riêng (src/fuzzy/fuzzy_type1.c) + body node + gravity comp.
add_executable(fuzzy_node src/fuzzy_node.cpp src/gravity_comp.cpp src/fuzzy/fuzzy_type1.c)
target_include_directories(fuzzy_node PRIVATE include ${CMAKE_CURRENT_SOURCE_DIR}/src/fuzzy)
ament_target_dependencies(fuzzy_node rclcpp sensor_msgs interbotix_xs_msgs urdf)
target_link_libraries(fuzzy_node m ruckig::ruckig pinocchio::pinocchio)
install(TARGETS fuzzy_node DESTINATION lib/${PROJECT_NAME})
# Install scripts (rx150_fuzzy_gui.py sẽ được thay thế bởi rx150_tuning_gui.py từ common).
install(
  PROGRAMS
    scripts/rx150_fuzzy_gui.py
    scripts/rx150_fuzzy_bridge_test.py
    scripts/rx150_gripper_trajectory_bridge.py
    scripts/rx150_fuzzy_record.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# env-hooks đã được đăng ký bởi rx150_motion_common
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
