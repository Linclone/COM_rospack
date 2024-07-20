cmake_minimum_required(VERSION 3.0.2)
project(send_mc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -funroll-loops -fpic -Wall -Wno-sign-compare -Werror=return-type -g")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fpic")

catkin_package(
#  CATKIN_DEPENDS
#  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES segwayrmp
# CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall")

include_directories(
 include
  ${catkin_INCLUDE_DIRS}



)


## Declare a C++ library

add_executable(send_node 
src/send_mc.cpp
)
#add_executable(receive_node src/recieve_mc.cpp)

target_link_libraries(send_node 
${catkin_LIBRARIES}
)
#target_link_libraries(receive_nodee ${catkin_LIBRARIES})
