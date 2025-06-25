cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CarMotionState.msg"
  DEPENDENCIES 
  geometry_msgs
    geometry_msgs
    std_msgs
)

ament_package()
