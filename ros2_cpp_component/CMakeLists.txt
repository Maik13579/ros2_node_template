cmake_minimum_required(VERSION 3.8)
project(ros2_cpp_component)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(ros2_cpp_component_interfaces REQUIRED)

include_directories(
  include
)

# Talker component
add_library(talker_component SHARED
    src/talker.cpp
)
ament_target_dependencies(talker_component
  rclcpp
  rclcpp_components
  ros2_cpp_component_interfaces
)
# target_link_libraries(filter_node_component
#   ${PCL_LIBRARIES}
#)
rclcpp_components_register_nodes(talker_component
    "demo::Talker"
)
target_link_libraries(talker_component)
install(TARGETS
  talker_component
  DESTINATION lib/${PROJECT_NAME}
)


# Listener component
add_library(listener_component SHARED
    src/listener.cpp
)
ament_target_dependencies(listener_component
  rclcpp
  rclcpp_components
  ros2_cpp_component_interfaces
)
# target_link_libraries(filter_node_component
#   ${PCL_LIBRARIES}
#)
rclcpp_components_register_nodes(listener_component
    "demo::Listener"
)
target_link_libraries(listener_component)
install(TARGETS
  listener_component
  DESTINATION lib/${PROJECT_NAME}
)





# -------------------------------------------------------------------
# Common Installs
# -------------------------------------------------------------------
install(TARGETS
  talker_component
  listener_component
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()