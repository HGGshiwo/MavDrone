cmake_minimum_required(VERSION 3.8)
project(mavdrone_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/TakeOff.action
  action/Land.action
  action/FlyToLocation.action
  srv/RotateGimbal.srv
  srv/Pause.srv
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()