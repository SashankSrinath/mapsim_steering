cmake_minimum_required(VERSION 3.5)
project(mapsim_steering)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bike_kinematics src/bike_kinematics.cpp)

ament_target_dependencies(
  bike_kinematics rclcpp std_msgs geometry_msgs sensor_msgs)
  
add_executable(two_steering_kinematics src/two_steering_kinematics.cpp)

ament_target_dependencies(
  two_steering_kinematics rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS bike_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS two_steering_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}) 
   
install(DIRECTORY launch urdf maps
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
