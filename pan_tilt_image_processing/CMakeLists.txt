cmake_minimum_required(VERSION 2.8.3)
project(pan_tilt_image_processing)

find_package(catkin REQUIRED COMPONENTS 
  actionlib
  actionlib_msgs 
  message_generation
  cv_bridge 
  image_transport 
  roscpp 
  sensor_msgs 
  std_msgs 
  pan_tilt_camera_teleop
)
#find_package(pan_tilt_camera_teleop REQUIRED)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  #Focusing.srv
  InfinityFocus.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
    AutoFocus.action
#   Action1.action
#   Action2.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)


catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS    
   actionlib_msgs
)
#add_definitions(${pan_tilt_camera_teleop_DEFINITIONS})
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${pan_tilt_camera_teleop_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
#LINK_DIRECTORIES(${catkin_LIBRARY_DIRS} ${pan_tilt_camera_teleop_LIBRARY_DIRS})


add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES})# ${pan_tilt_camera_teleop_LIBRARIES})
add_executable(auto_focus src/auto_focus.cpp)
target_link_libraries(auto_focus ${catkin_LIBRARIES})
add_executable(show_pan_tilt_image src/show_pan_tilt_image.cpp)
target_link_libraries(show_pan_tilt_image ${catkin_LIBRARIES})

add_executable(auto_focus_server src/auto_focus_server.cpp)
target_link_libraries(auto_focus_server ${catkin_LIBRARIES})
add_dependencies(auto_focus_server pan_tilt_image_processing_gencpp)

add_executable(auto_focus_client src/auto_focus_client.cpp)
target_link_libraries(auto_focus_client ${catkin_LIBRARIES})
add_dependencies(auto_focus_client pan_tilt_image_processing_gencpp)

add_executable(infinity_focus_server src/infinity_focus_server.cpp)
target_link_libraries(infinity_focus_server ${catkin_LIBRARIES})
add_dependencies(infinity_focus_server pan_tilt_image_processing_gencpp)

add_executable(infinity_focus_client src/infinity_focus_client.cpp)
target_link_libraries(infinity_focus_client ${catkin_LIBRARIES})
add_dependencies(infinity_focus_client pan_tilt_image_processing_gencpp)


#install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
