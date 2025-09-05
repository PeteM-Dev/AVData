cmake_minimum_required(VERSION 2.8.3)
project(ford_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  velodyne_msgs
  rostest
)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  add_rostest(test/test_extrinsics_broadcast.test)
  add_rostest(test/test_multi_lidar_integration.test)
  
  catkin_add_nosetests(test/test_extrinsics_broadcast.py)
endif()

catkin_install_python(PROGRAMS
  scripts/extrinsics_broadcaster.py
  scripts/bag_to_csv.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

