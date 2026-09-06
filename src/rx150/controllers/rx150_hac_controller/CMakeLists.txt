cmake_minimum_required(VERSION 3.5)
project(rx150_hac_controller)

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
find_package(std_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(ruckig CONFIG REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)

add_executable(hac_node src/hac_node.cpp src/gravity_comp.cpp src/hac/hac.c)
target_include_directories(hac_node PRIVATE include ${CMAKE_CURRENT_SOURCE_DIR}/src/hac)
ament_target_dependencies(hac_node rclcpp sensor_msgs std_msgs interbotix_xs_msgs urdf)
target_link_libraries(hac_node m ruckig::ruckig pinocchio::pinocchio)

install(TARGETS hac_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Environment hooks are registered by rx150_motion_common.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
