cmake_minimum_required(VERSION 2.8.3)
project(pocketsphinx)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS std_msgs std_srvs)

catkin_package(
  DEPENDS std_msgs std_srvs)

install(DIRECTORY demo/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS)

install(DIRECTORY nodes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS)
