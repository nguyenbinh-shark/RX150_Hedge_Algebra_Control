cmake_minimum_required(VERSION 3.8)
project(rx150_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Node nhận diện (Python, chạy như program — giống pattern rx150_fuzzy_controller).
# Pick-place đã chuyển sang package riêng rx150_pick_place (Layer 2).
install(
  PROGRAMS
    scripts/yolo_detector_node.py
    scripts/hand_gesture_node.py
    scripts/test_perception_modules.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared utils module để node có thể import.
install(
  FILES
    scripts/rx150_perception_utils.py
    scripts/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config models docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Install requirements.txt để user có thể pip install các pip-only deps (ultralytics, mediapipe).
install(
  FILES requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
