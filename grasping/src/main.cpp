/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

/* Modified by: Thomas Harrison */

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Pose.h>

#include <geometry_msgs/PoseArray.h>

#include <iostream>

#include <string>

std::string frame_id = "base_link";

void openGripper(trajectory_msgs::JointTrajectory &posture)
{
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "r_gripper_finger_joint";
    posture.joint_names[1] = "l_gripper_finger_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.045; // open gripper position (50mm in each direction)
    posture.points[0].positions[1] = 0.045;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
    // BEGIN_SUB_TUTORIAL closed_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "r_gripper_finger_joint";
    posture.joint_names[1] = "l_gripper_finger_joint";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface &move_group, std::string object, const geometry_msgs::Pose &bottle)
{
    // BEGIN_SUB_TUTORIAL pick1
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
    // of the cube). |br|
    // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
    // extra padding)

    // DEFINE POSE OF THE FETCH ROBOT ARM

    grasps[0].grasp_pose.header.frame_id = frame_id;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    grasps[0].grasp_pose.pose = bottle;
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x += -0.2; // grasps[0].grasp_pose.pose.position.x = grasps[0].grasp_pose.pose.position.x -0.2;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = frame_id;
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = frame_id;
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.x = -0.5;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);
    // // END_SUB_TUTORIAL
    // move_group.move();

    // moveit_msgs::MoveGroupGoal goal.

    // // BEGIN_SUB_TUTORIAL pick2
    // // Setting posture of eef during grasp
    // // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);

    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    //move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick(object, grasps);
    //END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface &group, std::string object, const geometry_msgs::Pose &bar)
{
    // BEGIN_SUB_TUTORIAL place
    // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
    // location in
    // verbose mode." This is a known issue and we are working on fixing it. |br|
    // Create a vector of placings to be attempted, currently only creating single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = frame_id;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    place_location[0].place_pose.pose = bar;
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    // // Setting pre-place approach
    // // ++++++++++++++++++++++++++
    // /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = frame_id;
    // /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = 1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // // Setting post-grasp retreat
    // // ++++++++++++++++++++++++++
    // /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = frame_id;
    // /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.x = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    //group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    group.place(object, place_location);
    // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, const geometry_msgs::Pose &bottle)
{
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.04;
    collision_objects[0].primitives[0].dimensions[1] = 0.04;
    collision_objects[0].primitives[0].dimensions[2] = 0.04;

    /* Define the pose of the object. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0] = bottle;

    // END_SUB_TUTORIAL

    // ADD ALL THE COLLISION OBJECTS/OBSTACLES HERE

    collision_objects[0].operation = collision_objects[0].ADD;

    // Add the first table where the cube will originally be kept.
    collision_objects[1].id = "table1";
    collision_objects[1].header.frame_id = "base_link";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.2;
    collision_objects[1].primitives[0].dimensions[1] = 0.4;
    collision_objects[1].primitives[0].dimensions[2] = 0.1;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.8;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = 0.95;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[2].id = "table2";
    collision_objects[2].header.frame_id = "base_link";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.4;
    collision_objects[2].primitives[0].dimensions[1] = 0.2;
    collision_objects[2].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.3;
    collision_objects[2].primitive_poses[0].position.y = 0.5;
    collision_objects[2].primitive_poses[0].position.z = 0.4;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

    // planning_scene_interface.removeCollisionObjects(collision_objects);

    // moveit_msgs::AttachedCollisionObject attached_object;

    // // Note that attaching an object to the robot requires
    // // the corresponding operation to be specified as an ADD operation.
    // attached_object.object.operation = attached_object.object.ADD;

    // // Since we are attaching the object to the robot hand to simulate picking up the object,
    // // we want the collision checker to ignore collisions between the object and the robot hand.
    // attached_object.touch_links = std::vector<std::string>{"gripper", "r_gripper_finger_joint", "l_gripper_finger_joint"};
}

void addBottleObject(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, std::string name, geometry_msgs::Pose &bottle)
{
    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = "base_link";
    collision_object.id = name;

    /* Define the primitive and its dimensions. */
    collision_object.primitives.resize(1);
    collision_object.primitives[0].type = collision_object.primitives[0].BOX;
    collision_object.primitives[0].dimensions.resize(3);
    collision_object.primitives[0].dimensions[0] = 0.07;
    collision_object.primitives[0].dimensions[1] = 0.07;
    collision_object.primitives[0].dimensions[2] = 0.20;

    /* Define the pose of the object. */
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0] = bottle;
    collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObject(collision_object);
}

void addObstacles(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, geometry_msgs::Pose &fetch)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(7);

    /////////////////////////////////////////////////////////////////////

    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].id = "left_bench";

    double left_bench_z = 1.02;

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.9;
    collision_objects[0].primitives[0].dimensions[1] = 1.6;
    collision_objects[0].primitives[0].dimensions[2] = left_bench_z;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);

    collision_objects[0].primitive_poses[0] = fetch; // grasps[0].grasp_pose.pose.position.x = grasps[0].grasp_pose.pose.position.x -0.2;

    collision_objects[0].primitive_poses[0].position.x += 1.2;
    collision_objects[0].primitive_poses[0].position.y += 3;
    collision_objects[0].primitive_poses[0].position.z += left_bench_z / 2;
    //collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[0].operation = collision_objects[0].ADD;

    ////////////////////////////////////////////////////////////////////

    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].id = "right_bench";

    double right_bench_z = 1.02;

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.9;
    collision_objects[1].primitives[0].dimensions[1] = 1.6;
    collision_objects[1].primitives[0].dimensions[2] = left_bench_z;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);

    collision_objects[1].primitive_poses[0] = fetch; // grasps[0].grasp_pose.pose.position.x = grasps[0].grasp_pose.pose.position.x -0.2;

    collision_objects[1].primitive_poses[0].position.x += 1.2;
    collision_objects[1].primitive_poses[0].position.y += 0;
    collision_objects[1].primitive_poses[0].position.z += left_bench_z / 2;
    //collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[1].operation = collision_objects[1].ADD;

    /////////////////////////////////////////////////////////////////////

    collision_objects[2].header.frame_id = "base_link";
    collision_objects[2].id = "bottom_shelf";

    double bottom_shelf_z = 0.1;

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.4;
    collision_objects[2].primitives[0].dimensions[1] = 4.8;
    collision_objects[2].primitives[0].dimensions[2] = bottom_shelf_z;

    /* Define the pose of the table. */
    collision_objects[2].primitive_poses.resize(1);

    collision_objects[2].primitive_poses[0] = fetch; // grasps[0].grasp_pose.pose.position.x = grasps[0].grasp_pose.pose.position.x -0.2;

    collision_objects[2].primitive_poses[0].position.x += 1.8;
    collision_objects[2].primitive_poses[0].position.y += 1.5;
    collision_objects[2].primitive_poses[0].position.z += 1.06 - (bottom_shelf_z / 2);
    //collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[2].operation = collision_objects[2].ADD;

    /////////////////////////////////////////////////////////////////////

    collision_objects[3].header.frame_id = "base_link";
    collision_objects[3].id = "top_shelf";

    double top_shelf_z = 1;

    /* Define the primitive and its dimensions. */
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.4;
    collision_objects[3].primitives[0].dimensions[1] = 4.8;
    collision_objects[3].primitives[0].dimensions[2] = top_shelf_z;

    /* Define the pose of the table. */
    collision_objects[3].primitive_poses.resize(1);

    collision_objects[3].primitive_poses[0] = fetch; // grasps[0].grasp_pose.pose.position.x = grasps[0].grasp_pose.pose.position.x -0.2;

    collision_objects[3].primitive_poses[0].position.x += 1.8;
    collision_objects[3].primitive_poses[0].position.y += 1.5;
    collision_objects[3].primitive_poses[0].position.z += (1.06 + 0.3) + (top_shelf_z / 2);
    //collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[3].operation = collision_objects[3].ADD;

    /////////////////////////////////////////////////////////////////////

    collision_objects[4].header.frame_id = "base_link";
    collision_objects[4].id = "back_shelf";

    double back_shelf_z = 2.36;

    /* Define the primitive and its dimensions. */
    collision_objects[4].primitives.resize(1);
    collision_objects[4].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[4].primitives[0].dimensions.resize(3);
    collision_objects[4].primitives[0].dimensions[0] = 0.05;
    collision_objects[4].primitives[0].dimensions[1] = 4.8;
    collision_objects[4].primitives[0].dimensions[2] = back_shelf_z;

    /* Define the pose of the table. */
    collision_objects[4].primitive_poses.resize(1);

    collision_objects[4].primitive_poses[0] = fetch; // grasps[0].grasp_pose.pose.position.x = grasps[0].grasp_pose.pose.position.x -0.2;

    collision_objects[4].primitive_poses[0].position.x += 1.8 + (0.2 - 0.025);
    collision_objects[4].primitive_poses[0].position.y += 1.5;
    collision_objects[4].primitive_poses[0].position.z += back_shelf_z / 2;
    //collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[4].operation = collision_objects[4].ADD;

    /////////////////////////////////////////////////////////////////////

    collision_objects[5].header.frame_id = "base_link";
    collision_objects[5].id = "left_shelf";

    double left_shelf_z = 0.3;

    /* Define the primitive and its dimensions. */
    collision_objects[5].primitives.resize(1);
    collision_objects[5].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[5].primitives[0].dimensions.resize(3);
    collision_objects[5].primitives[0].dimensions[0] = 0.4;
    collision_objects[5].primitives[0].dimensions[1] = 1.9;
    collision_objects[5].primitives[0].dimensions[2] = left_shelf_z;

    /* Define the pose of the table. */
    collision_objects[5].primitive_poses.resize(1);

    collision_objects[5].primitive_poses[0] = fetch; // grasps[0].grasp_pose.pose.position.x = grasps[0].grasp_pose.pose.position.x -0.2;

    collision_objects[5].primitive_poses[0].position.x += 1.8;
    collision_objects[5].primitive_poses[0].position.y += 2.9;
    collision_objects[5].primitive_poses[0].position.z += 1.06 + (left_shelf_z / 2);
    //collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[5].operation = collision_objects[5].ADD;

    ////////////////////////////////////////////////////////////////////

    collision_objects[6].header.frame_id = "base_link";
    collision_objects[6].id = "right_shelf";

    double right_shelf_z = 0.3;

    /* Define the primitive and its dimensions. */
    collision_objects[6].primitives.resize(1);
    collision_objects[6].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[6].primitives[0].dimensions.resize(3);
    collision_objects[6].primitives[0].dimensions[0] = 0.4;
    collision_objects[6].primitives[0].dimensions[1] = 1.9;
    collision_objects[6].primitives[0].dimensions[2] = left_shelf_z;

    /* Define the pose of the table. */
    collision_objects[6].primitive_poses.resize(1);

    collision_objects[6].primitive_poses[0] = fetch; // grasps[0].grasp_pose.pose.position.x = grasps[0].grasp_pose.pose.position.x -0.2;

    collision_objects[6].primitive_poses[0].position.x += 1.8;
    collision_objects[6].primitive_poses[0].position.y += 0.1;
    collision_objects[6].primitive_poses[0].position.z += 1.06 + (left_shelf_z / 2);
    //collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[6].operation = collision_objects[5].ADD;

    ////////////////////////////////////////////////////////////////////

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

   geometry_msgs::Pose fetch;
    fetch.position.x = -(0.49998);
    fetch.position.y = -(1.5);
    fetch.position.z = -(0.004772);
    fetch.orientation.w = 1;
   
    geometry_msgs::Pose bottle;
    bottle.position.x += (0.7);
    bottle.position.y += (0);
    bottle.position.z += (1.4);

    geometry_msgs::Pose bar;
    bar.position.x = 0.5;
    bar.position.y = 0.3;
    bar.position.z = 0.8;

    ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface arm("arm_with_torso");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");

    gripper.setPlanningTime(45.0);

    //addCollisionObjects(planning_scene_interface, bottle);
    addObstacles(planning_scene_interface, fetch);
    addBottleObject(planning_scene_interface, "bottle", bottle);

    ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

    pick(arm, "bottle", bottle);

    ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

    place(arm, "bottle", bar);

    ros::shutdown();

    return 0;
}