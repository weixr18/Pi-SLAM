cmake_minimum_required(VERSION 3.0.2)
project(dumbpi)

# catkin
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 std_msgs
 genmsg
)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)
## Declare a catkin package
catkin_package()

## Build cpps
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(controller src/controller.cpp)
target_link_libraries(controller wiringPi ${catkin_LIBRARIES})
add_dependencies(controller dumbpi_generate_messages_cpp)

add_executable(keyboard src/keyboard.cpp)
target_link_libraries(keyboard ${catkin_LIBRARIES})
add_dependencies(keyboard dumbpi_generate_messages_cpp)
