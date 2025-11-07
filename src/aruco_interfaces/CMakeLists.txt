cmake_minimum_required(VERSION 3.8)
project(aruco_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(aruco_markers_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GetArucoPose.action"
  DEPENDENCIES std_msgs geometry_msgs
)

install(DIRECTORY
  action
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)
ament_package()
