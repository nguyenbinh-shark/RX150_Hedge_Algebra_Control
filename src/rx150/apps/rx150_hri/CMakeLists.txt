cmake_minimum_required(VERSION 3.8)
project(rx150_hri)

# Chức năng HRI tự chứa: hri_task (logic đọc nhận diện + lựa chọn + định trình tự)
# phát lệnh cho hri_motion (executor: MoveIt + IK-oracle + gripper).
# Convention workspace: ament_cmake (KHÔNG ament_python) + install(PROGRAMS)
# để ros2 discovery thấy package (xem rx150_perception / rx150_pick_place).
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/hri_task_node.py
    scripts/hri_motion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared module để node có thể import (pattern rx150_perception_utils).
install(
  FILES
    scripts/hri_common.py
    scripts/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
