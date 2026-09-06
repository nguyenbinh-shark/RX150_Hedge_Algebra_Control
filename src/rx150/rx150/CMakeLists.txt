cmake_minimum_required(VERSION 3.8)
project(rx150)

# Metapackage: không build gì, chỉ gom phụ thuộc (xem package.xml).
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
