cmake_minimum_required(VERSION 3.8)
project(ros2_cpp_component_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
    "msg/CustomMsg.msg"
)

set(srv_files
    "srv/CustomService.srv"
)

set(action_files
    "action/CustomAction.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)


ament_package()
