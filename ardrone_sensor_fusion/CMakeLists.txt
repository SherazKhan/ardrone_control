cmake_minimum_required(VERSION 2.8.3)
project(ardrone_sensor_fusion)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_ardrone_sensor_fusion.test)
endif()
