cmake_minimum_required(VERSION 2.8.3)
project(pan_tilt_image_processing)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp sensor_msgs std_msgs pan_tilt_camera_teleop)
#find_package(pan_tilt_camera_teleop REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS    
   cv_bridge
   image_transport
   roscpp
   sensor_msgs
   std_msgs
   pan_tilt_camera_teleop
)
#add_definitions(${pan_tilt_camera_teleop_DEFINITIONS})
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${pan_tilt_camera_teleop_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
#LINK_DIRECTORIES(${catkin_LIBRARY_DIRS} ${pan_tilt_camera_teleop_LIBRARY_DIRS})


add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES})# ${pan_tilt_camera_teleop_LIBRARIES})
add_executable(auto_focus src/auto_focus.cpp)
target_link_libraries(auto_focus ${catkin_LIBRARIES})



#install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
