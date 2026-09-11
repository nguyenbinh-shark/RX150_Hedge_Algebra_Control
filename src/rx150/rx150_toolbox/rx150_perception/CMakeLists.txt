cmake_minimum_required(VERSION 3.8)
project(rx150_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Node nhận diện
install(
  PROGRAMS
    scripts/hand_gesture_node.py
    scripts/rack_calib_node.py
    scripts/yolo_tube_detector_node.py
    test/test_rack_calib.py
    test/test_yolo_tube_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Install requirements.txt để user có thể pip install các pip-only deps.
install(
  FILES requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
