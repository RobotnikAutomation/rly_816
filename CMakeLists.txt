cmake_minimum_required(VERSION 2.8.3)
project(phidget_load)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs robotnik_msgs)
find_package(Boost REQUIRED)

catkin_package(
  
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS std_msgs robotnik_msgs
   DEPENDS messages_runtime std_msgs)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


