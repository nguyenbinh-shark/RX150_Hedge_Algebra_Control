cmake_minimum_required(VERSION 3.8)
project(rx150_pick_place)

# Layer 2: quyết định pick-place qua MoveIt (Python-only).
# Convention workspace: ament_cmake (KHÔNG ament_python) + install(PROGRAMS)
# để ros2 discovery thấy package (xem rx150_perception).
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Thư viện dùng chung (kinematics / motion / gripper / scene / skills / …) được
# cài NGAY CẠNH các script, chứ không qua ament_python_install_package: bản
# setuptools trên máy này (84.0) không hợp với packaging 21.3 nên bước build egg
# của ament_cmake_python fail (canonicalize_version: strip_trailing_zero).
# Python luôn đặt thư mục chứa script vào sys.path[0], nên
# `from rx150_pick_place.skills import …` trong scripts/*.py vẫn chạy.
install(
  DIRECTORY ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

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

# Đưa lib/rx150_pick_place vào PYTHONPATH khi source install/setup.bash.
# (sys.path[0] không dùng được: colcon --symlink-install làm script thành
# symlink, Python lấy thư mục THẬT của script là src/…/scripts/.)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/rx150_pick_place_pythonpath.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Kiểm chứng IK/FK giải tích — chạy được không cần robot lẫn MoveIt.
  ament_add_pytest_test(test_kinematics test/test_kinematics.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
