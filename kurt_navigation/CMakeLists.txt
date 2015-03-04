cmake_minimum_required(VERSION 2.8)
project(kurt_navigation)

find_package(catkin REQUIRED COMPONENTS
             move_base amcl gmapping frontier_exploration)

catkin_package(CATKIN_DEPENDS move_base amcl gmapping frontier_exploration)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
