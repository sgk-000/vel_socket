cmake_minimum_required(VERSION 2.8.3)
project(vel_socket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES socket_sample
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/teleop_client.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
