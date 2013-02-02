cmake_minimum_required(VERSION 2.8.3)
project(pocketsphinx)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY nodes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
