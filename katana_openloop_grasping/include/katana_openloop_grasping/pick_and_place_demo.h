/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2010  University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * pick_and_place_demo.h
 *
 *  Created on: 31.01.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef PICK_AND_PLACE_DEMO_H_
#define PICK_AND_PLACE_DEMO_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <move_arm_msgs/MoveArmAction.h>

#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <object_manipulation_msgs/GraspStatus.h>

#include <collision_environment_msgs/MakeStaticCollisionMapAction.h>

typedef object_manipulation_msgs::GraspHandPostureExecutionGoal GHPEG;

namespace katana_openloop_grasping
{

static const size_t NUM_JOINTS = 5;

static const double GRIPPER_OPEN_ANGLE = 0.30;
static const double GRIPPER_CLOSED_ANGLE = -0.44;

class PickAndPlaceDemo
{
public:
  PickAndPlaceDemo();
  virtual ~PickAndPlaceDemo();

  void loop();

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm_;
  actionlib::SimpleActionClient<object_manipulation_msgs::GraspHandPostureExecutionAction> gripper_;
  actionlib::SimpleActionClient<collision_environment_msgs::MakeStaticCollisionMapAction> make_static_collision_map_;
  ros::ServiceClient grasp_status_client_;

  bool move_to_joint_goal(std::vector<motion_planning_msgs::JointConstraint> joint_constraints);
  bool send_gripper_action(int32_t goal_type);
  bool make_static_collision_map();
  bool query_grasp_status();

};

}

#endif /* PICK_AND_PLACE_DEMO_H_ */
