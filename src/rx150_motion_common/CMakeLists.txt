cmake_minimum_required(VERSION 3.8)
project(rx150_motion_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python scripts (bridge + GUI + tuning/hiệu chuẩn HAC)
install(PROGRAMS
  scripts/rx150_trajectory_bridge.py
  scripts/rx150_tuning_gui.py
  scripts/tuning_lib.py
  scripts/rx150_tuning_session.py
  scripts/rx150_run_compare.py
  scripts/rx150_gravity_id.py
  scripts/rx150_friction_id.py
  DESTINATION lib/${PROJECT_NAME})

# Install shared config (motor yaml, sensors_3d)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install env-hooks (geometric_shapes_shim.sh)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/geometric_shapes_shim.sh")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
