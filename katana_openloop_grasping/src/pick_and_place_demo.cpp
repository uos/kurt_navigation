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
 * pick_and_place_demo.cpp
 *
 *  Created on: 31.01.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include <katana_openloop_grasping/pick_and_place_demo.h>

namespace katana_openloop_grasping
{

PickAndPlaceDemo::PickAndPlaceDemo() :
  move_arm_("move_arm", true), gripper_("gripper_grasp_posture_controller", true),
      make_static_collision_map_("make_static_collision_map", true)
{
  ros::NodeHandle nh;

  move_arm_.waitForServer();
  ROS_INFO("Connected to move arm action server");

  gripper_.waitForServer();
  ROS_INFO("Connected to gripper action server");

  // make_static_collision_map_.waitForServer();
  // ROS_INFO("Connected to make_static_collision_map action server");

  grasp_status_client_ = nh.serviceClient<object_manipulation_msgs::GraspStatus> ("gripper_grasp_status");
  if (!grasp_status_client_.waitForExistence(ros::Duration(10.0)))
  {
    ROS_FATAL("Could not connect to gripper grasp status service");
    return;
  }
  else
  {
    ROS_INFO("Connected to gripper grasp status service");
  }

}

PickAndPlaceDemo::~PickAndPlaceDemo()
{
}

void PickAndPlaceDemo::loop()
{

  std::vector<std::string> names(NUM_JOINTS);
  names[0] = "katana_motor1_pan_joint";
  names[1] = "katana_motor2_lift_joint";
  names[2] = "katana_motor3_lift_joint";
  names[3] = "katana_motor4_lift_joint";
  names[4] = "katana_motor5_wrist_roll_joint";

  std::vector<motion_planning_msgs::JointConstraint> joint_constraints(NUM_JOINTS);

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joint_constraints[i].joint_name = names[i];
    joint_constraints[i].tolerance_below = 0.1;
    joint_constraints[i].tolerance_above = 0.1;
  }

  while (nh_.ok())
  {
    bool success;

    // TODO: To make the grasping work, we need to add a known object to the environment and attach
    // it to the gripper, or else disable the collision checks for the fingers

    ROS_INFO("Sending PRE_GRASP");
    success = send_gripper_action(GHPEG::PRE_GRASP);
    success &= query_grasp_status();
    if (!success)
      break;

    //success = make_static_collision_map();
    //if (!success)
    //  break;

    //  == position 1 ==
    joint_constraints[0].position = 0.029225487499999758;
    joint_constraints[1].position = 0.11312164000000005;
    joint_constraints[2].position = -0.61444545070000012;
    joint_constraints[3].position = 1.3272473432000003;
    joint_constraints[4].position = 0.13708330500000043;

    ROS_INFO("Moving to goal 1");
    success = move_to_joint_goal(joint_constraints);
    if (!success)
      break;

    ros::Duration(1.0).sleep();
    ROS_INFO("Sending GRASP");
    success = send_gripper_action(GHPEG::GRASP);
    success &= query_grasp_status();
    if (!success)
      break;

    //success = make_static_collision_map();
    //if (!success)
    //  break;

    //  == position 2 ==
    joint_constraints[0].position = 1.0;
    joint_constraints[1].position = 0.11312164000000005;
    joint_constraints[2].position = -0.61444545070000012;
    joint_constraints[3].position = 1.3272473432000003;
    joint_constraints[4].position = -0.038843444999999921;

    ROS_INFO("Moving to goal 2");
    success = move_to_joint_goal(joint_constraints);
    if (!success)
      break;

    ros::Duration(1.0).sleep();
    ROS_INFO("Sending RELEASE");
    success = send_gripper_action(GHPEG::RELEASE);
    success &= query_grasp_status();
    if (!success)
      break;
  }
}

bool PickAndPlaceDemo::move_to_joint_goal(std::vector<motion_planning_msgs::JointConstraint> joint_constraints)
{
  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");

  goal.motion_plan_request.goal_constraints.joint_constraints = joint_constraints;

  bool finished_within_time = false;
  move_arm_.sendGoal(goal);
  finished_within_time = move_arm_.waitForResult(ros::Duration(40.0));
  if (!finished_within_time)
  {
    move_arm_.cancelGoal();
    ROS_INFO("Timed out achieving goal!");
    return false;
  }
  else
  {
    actionlib::SimpleClientGoalState state = move_arm_.getState();
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (success)
      ROS_INFO("Action finished: %s", state.toString().c_str());
    else
      ROS_INFO("Action failed: %s", state.toString().c_str());

    return success;
  }
}

bool PickAndPlaceDemo::send_gripper_action(int32_t goal_type)
{

  GHPEG goal;

  switch (goal_type)
  {
    case GHPEG::GRASP:
      goal.grasp.grasp_posture.name.push_back("dummy_name");
      goal.grasp.grasp_posture.position.push_back(GRIPPER_CLOSED_ANGLE);
      // leave velocity and effort empty
      break;

    case GHPEG::PRE_GRASP:
      goal.grasp.pre_grasp_posture.name.push_back("dummy_name");
      goal.grasp.pre_grasp_posture.position.push_back(GRIPPER_OPEN_ANGLE);
      // leave velocity and effort empty
      break;

    case GHPEG::RELEASE:
      break;

    default:
      ROS_ERROR("unknown goal code (%d)", goal_type);
      return false;
  }

  goal.goal = goal_type;

  bool finished_within_time = false;
  gripper_.sendGoal(goal);
  finished_within_time = gripper_.waitForResult(ros::Duration(40.0));
  if (!finished_within_time)
  {
    gripper_.cancelGoal();
    ROS_WARN("Timed out achieving goal!");
    return false;
  }
  else
  {
    actionlib::SimpleClientGoalState state = gripper_.getState();
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (success)
      ROS_INFO("Action finished: %s",state.toString().c_str());
    else
      ROS_WARN("Action failed: %s",state.toString().c_str());

    return success;
  }
}

bool PickAndPlaceDemo::make_static_collision_map() {
  ROS_INFO("Making static collision map");

  collision_environment_msgs::MakeStaticCollisionMapGoal goal;
  goal.cloud_source = "tilt_scan_cloud_filtered";
  goal.number_of_clouds = 1;

  make_static_collision_map_.sendGoal(goal);

  if (!make_static_collision_map_.waitForResult(ros::Duration(30.0)))
  {
    ROS_ERROR("Timeout when waiting on make_static_collision_map");
    return false;
  }

  if (make_static_collision_map_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    return true;
  }else {
    ROS_ERROR("make_static_collision_map action failed!");
    return false;
  }
}


bool PickAndPlaceDemo::query_grasp_status()
{
  object_manipulation_msgs::GraspStatus srv;

  bool success = grasp_status_client_.call(srv);
  if (success)
    if (srv.response.is_hand_occupied)
      ROS_INFO("Gripper is holding something");
    else
      ROS_INFO("Gripper is empty");
  else
    ROS_ERROR("Failed to call service gripper_grasp_status");

  return success;
}

} // namespace katana_openloop_grasping

int main(int argc, char **argv)
{
  // TODO: rosservice call /collision_map_self_occ_node/reset

  ros::init(argc, argv, "pick_and_place_demo");
  katana_openloop_grasping::PickAndPlaceDemo demo;

  demo.loop();

  ros::shutdown();
}
