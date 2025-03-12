cmake_minimum_required(VERSION 3.5)
project(prophesee_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# OpenCVの設定（必要なコンポーネントがあれば指定）
find_package(OpenCV REQUIRED)
find_package(MetavisionSDK COMPONENTS driver REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Driver（publisher）の実行ファイルターゲット
ament_auto_add_executable(prophesee_ros2_driver
  src/prophesee_ros2_publisher.cpp
)

target_link_libraries(prophesee_ros2_driver
  MetavisionSDK::driver
  ${OpenCV_LIBS}
)

# Viewer の実行ファイルターゲット（cd_frame_generatorも含む）
ament_auto_add_executable(prophesee_ros2_viewer
  src/prophesee_ros2_viewer.cpp
  src/cd_frame_generator.cpp
)

target_link_libraries(prophesee_ros2_viewer
  MetavisionSDK::driver
  ${OpenCV_LIBS}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
