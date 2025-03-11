cmake_minimum_required(VERSION 3.5)
project(prophesee_event_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Event.msg"
  "msg/EventArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_auto_package()
