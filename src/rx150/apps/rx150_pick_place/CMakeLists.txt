cmake_minimum_required(VERSION 3.8)
project(rx150_pick_place)

# Tầng Application: chỉ có node quyết định + config + launch.
# Thư viện dùng chung (kinematics/motion/gripper/scene/skills/status) đã chuyển sang
# package rx150_modules (Application Support) — package này chỉ `from rx150_modules…`.
# Convention workspace: ament_cmake (KHÔNG ament_python) + install(PROGRAMS) để
# ros2 discovery thấy package (xem rx150_perception).
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/pick_place_moveit_node.py
    scripts/tube_rack_node.py
    scripts/rx150_reach_check.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
