cmake_minimum_required(VERSION 3.8)
project(rx150_pick_place)

# Layer 2: node quyết định pick-place qua MoveIt (Python-only).
# Convention workspace: ament_cmake (KHÔNG ament_python) + install(PROGRAMS)
# để ros2 discovery thấy package (xem rx150_perception).
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/pick_place_moveit_node.py
    scripts/tube_rack_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
