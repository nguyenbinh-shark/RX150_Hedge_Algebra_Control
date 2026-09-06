cmake_minimum_required(VERSION 3.5)
project(rx150_ff_controller)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
# TOTG online (profile vận tốc/gia tốc trước bộ fuzzy). apt: ros-humble-ruckig.
find_package(ruckig CONFIG REQUIRED)
# KHÔNG pinocchio/urdf — feedforward là gain động học (Kv·q̇ + Ka·q̈), không cần dynamic model.

add_executable(ff_node src/ff_node.cpp src/fuzzy/fuzzy_type1.c)
target_include_directories(ff_node PRIVATE include ${CMAKE_CURRENT_SOURCE_DIR}/src/fuzzy)
ament_target_dependencies(ff_node rclcpp sensor_msgs interbotix_xs_msgs)
target_link_libraries(ff_node m ruckig::ruckig)   # fuzzy_type1.c dùng expf/fabsf/powf
install(TARGETS ff_node DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/ff_trajectory_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Shim libgeometric_shapes 2.3.2 cho move_group (parity fuzzy_controller).
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/geometric_shapes_shim.sh"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
