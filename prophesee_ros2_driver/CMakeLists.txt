cmake_minimum_required(VERSION 3.5)
project(prophesee_ros2_driver)

# C++17を利用
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O3)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# OpenCVの設定（必要なコンポーネントがあれば指定）
find_package(OpenCV REQUIRED)
# MetavisionSDKのstreamコンポーネントを取得
find_package(MetavisionSDK COMPONENTS stream REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Driver（publisher）の実行ファイルターゲット
ament_auto_add_executable(prophesee_ros2_driver
  src/prophesee_ros2_publisher.cpp
)

target_link_libraries(prophesee_ros2_driver
  MetavisionSDK::stream
  ${OpenCV_LIBS}
)

# Viewer の実行ファイルターゲット（cd_frame_generatorも含む）
ament_auto_add_executable(prophesee_ros2_viewer
  src/prophesee_ros2_viewer.cpp
  src/cd_frame_generator.cpp
)

target_link_libraries(prophesee_ros2_viewer
  MetavisionSDK::stream
  ${OpenCV_LIBS}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
