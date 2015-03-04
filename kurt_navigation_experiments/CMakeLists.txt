cmake_minimum_required(VERSION 2.8)
project(kurt_navigation_experiments)

find_package(catkin REQUIRED COMPONENTS
             kurt_navigation
             kurt_bringup
             kurt_gazebo)

catkin_package(CATKIN_DEPENDS kurt_navigation kurt_bringup kurt_gazebo)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY maps/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps)
