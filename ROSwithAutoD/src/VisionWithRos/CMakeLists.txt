cmake_minimum_required(VERSION 3.0.2)
project(VisionWithRos)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport 
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES VisionWithRos
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(LaneDetect src/LaneDetect.cpp)
target_link_libraries(LaneDetect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)