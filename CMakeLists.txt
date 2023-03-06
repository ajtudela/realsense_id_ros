cmake_minimum_required(VERSION 3.8)
project(realsense_id_ros)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################
## Find  dependencies                         ##
################################################
## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rsid "0.25.0" EXACT REQUIRED PATHS "/opt/RealSenseID/lib64/cmake")

# RealSense ID
SET(rsid_INCLUDE_DIRS "/opt/RealSenseID/include")
SET(rsid_LIBS_DIRS "/opt/RealSenseID/lib64")
FIND_LIBRARY(rsid_LIBRARIES rsid ${rsid_LIBS_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################
set(msg_files
  "msg/Face.msg"
  "msg/FaceArray.msg"
)
set(srv_files
  "srv/Authenticate.srv"
  "srv/DeviceInfo.srv"
  "srv/Enroll.srv"
  "srv/RemoveUser.srv"
  "srv/RemoveAllUsers.srv"
  "srv/QueryUsersId.srv"
  "srv/StartAuthenticationLoop.srv"
  "srv/StopAuthenticationLoop.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs vision_msgs
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${rsid_INCLUDE_DIRS}
)

# executable node
add_executable(realsense_id_ros_node
  src/realsense_id_ros.cpp
  src/realsense_id_ros_node.cpp
)
ament_target_dependencies(realsense_id_ros_node
  rclcpp
  std_msgs 
  sensor_msgs
  vision_msgs
  cv_bridge
  nav2_util
)
target_link_libraries(realsense_id_ros_node
  ${rsid_LIBRARIES}
  #${cpp_typesupport_target} # Add if Humble
)
# Deprecated as of Humble. Fix with the comment below
rosidl_target_interfaces(realsense_id_ros_node ${PROJECT_NAME} "rosidl_typesupport_cpp") # Remove if Humble
#rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp") # Add if Humble

#############
## Install ##
#############
install(TARGETS realsense_id_ros_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()