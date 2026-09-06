cmake_minimum_required(VERSION 3.8)
project(rx150_modules)

# Tầng Application Support: thư viện Python thuần, không có node.
find_package(ament_cmake REQUIRED)

# Cài thư viện vào lib/ chứ KHÔNG qua ament_python_install_package: setuptools 84
# trong ~/.local không hợp với packaging 21.3 của apt nên bước build egg của
# ament_cmake_python fail (canonicalize_version: strip_trailing_zero). Xem
# tools/build.sh — build bằng script đó thì cả hai đường đều chạy, nhưng cách cài
# này không phụ thuộc vào việc ai đó nhớ đặt PYTHONNOUSERSITE.
install(
  DIRECTORY ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Đưa lib/rx150_modules vào PYTHONPATH khi source install/setup.bash, để mọi app
# `from rx150_modules.… import …` được. (sys.path[0] không dùng được: với
# --symlink-install thì script là symlink, Python lấy thư mục THẬT của script.)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/rx150_modules_pythonpath.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Cả hai bài đều chạy được KHÔNG CẦN robot lẫn MoveIt.
  ament_add_pytest_test(test_kinematics test/test_kinematics.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_motion_recipe test/test_motion_recipe.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
