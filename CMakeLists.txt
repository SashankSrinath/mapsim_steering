cmake_minimum_required(VERSION 3.5)
project(mapsim_steering)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(fwd_kinematics src/fwd_kinematics.cpp)
ament_target_dependencies(
  fwd_kinematics rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS fwd_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch urdf maps
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
