cmake_minimum_required(VERSION 2.8.3)

project(realsense_sp)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# View the makefile commands during build
set(CMAKE_VERBOSE_MAKEFILE on)

# Set compile flags
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
# Flags executables
set(CMAKE_EXE_LINKER_FLAGS "-pie -z noexecstack -z relro -z now")
# Flags shared libraries
set(CMAKE_SHARED_LINKER_FLAGS "-z noexecstack -z relro -z now")

# OpenCV used for producing detection image frames.
find_package(OpenCV REQUIRED)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  dynamic_reconfigure
  cv_bridge
  message_filters
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  rostest
  realsense_camera
  tf
  
)

add_message_files(
  FILES
  Status.msg
  
)

add_service_files(
  FILES
  Reset.srv
 
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime
  LIBRARIES ${PROJECT_NAME}_nodelet
)

###########
## Build ##
###########
# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
# Declare a C++ library
add_library(${PROJECT_NAME}_nodelet
  src/realsense_sp_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelet
    realsense
    realsense_image
    realsense_slam
    SP_Core
    tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(${PROJECT_NAME}_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)


#############
## Install ##
#############
# Install nodelet library
install(TARGETS ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
