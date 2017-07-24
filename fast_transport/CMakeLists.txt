cmake_minimum_required(VERSION 2.8.3)
project(fast_transport)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    fast_transport_msgs
    message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    FastMsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
    DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
