cmake_minimum_required(VERSION 2.8.3)
project(pan_tilt_joy_control)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mar_robot_arm5e
  roscpp
  sensor_msgs
  pan_tilt_camera_teleop
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pan_tilt_joy_control
 CATKIN_DEPENDS
   mar_robot_arm5e
   roscpp 
   sensor_msgs
   pan_tilt_camera_teleop
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(PanTiltJoyController src/PanTiltJoyController.cpp)
target_link_libraries(PanTiltJoyController ${catkin_LIBRARIES})
