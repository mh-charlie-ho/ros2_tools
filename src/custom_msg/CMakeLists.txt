cmake_minimum_required(VERSION 3.8)
project(custom_msg)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB absolute_msg_files "msg/*.msg")
foreach(file IN LISTS absolute_msg_files)
  get_filename_component(file_name ${file} NAME)
  set(msg_files ${msg_files} "msg/${file_name}")
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package()
