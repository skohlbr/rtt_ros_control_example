cmake_minimum_required(VERSION 2.8.3)
project(hw_interface_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  roscpp
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES hw_interface_example
#  CATKIN_DEPENDS hardware_interface roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(hw_interface_example
  src/hw_interface_example.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(hw_interface_example
  ${catkin_LIBRARIES}
)

