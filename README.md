kurt_navigation
=============

This stack contains navigation components for KURT. Specifically, it provides: 

* config and launch files for the basic navigation behaviors (slam, map navigation) and the respective components (gmapping, amcl, etc.) (package `kurt_navigation`),
* launch files and map data to navigate KURT the labs of UOS (both in reality and gazebo) (package `kurt_navigation_experiments`)

Running kurt_navigation in Gazebo
-----------------------------------

1. Bring up KURT in Gazebo:

        LC_ALL=C roslaunch kurt_gazebo kurt_avz_world.launch

   This starts KURT in the "AVZ 5th floor" world. (The `LC_ALL` trick is
   necessary to work around
   [this bug](https://bitbucket.org/osrf/sdformat/issues/60/error-when-starting-gazebo-if-lc_numeric)
   on indigo.)

2. Start the navigation stack with AMCL:

        roslaunch kurt_navigation single_map_navigation.launch map_file:=$(rospack find uos_maps)/maps/avz5floor_gazebo.yaml

3. Run RViz:

        rosrun rviz rviz -d $(rospack find kurt_navigation)/rviz/navigation.rviz

   In RViz, click "2D Pose Estimate" and select the approximate position of the
   robot in the map.

4. Run teleop:

        rosrun uos_diffdrive_teleop uos_diffdrive_teleop_key

   Using teleop, drive the robot around a bit until it has a good localization
   (watch the AMCL Particle Swarm in RViz). Remember to keep the focus on the
   terminal window that runs `uos_diffdrive_teleop_key` while pressing buttons.
   It is important that the robot's localization is good before trying to pass
   any narrow doorways.

5. Give a goal to `move_base` by clicking the "2D Nav Goal" button in RViz and
   selecting a pose.


If you want to try [gmapping](http://wiki.ros.org/gmapping) instead of AMCL, run this instead of step 2:

    roslaunch kurt_navigation slam.launch

That launch file also starts
[frontier_exploration](http://wiki.ros.org/frontier_exploration). If you want
to try it, visualize the topics `exploration_polygon_marker` and
`/explore_server/explore_costmap/costmap` in RViz, draw a closed polygon using
the "Publish Point" button and click once inside the polygon.
