cmake_minimum_required(VERSION 2.8.3)
project(fast_transport_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
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