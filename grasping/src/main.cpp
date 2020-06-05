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

#include <iostream>

#include <string>

std::string frame_id = "gripper";

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

void pick(moveit::planning_interface::MoveGroupInterface &move_group)
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
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.45;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 1.2;

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
    move_group.pick("object", grasps);
    //END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

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
    collision_objects[0].primitive_poses[0].position.x = 0.65;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 1.18;
    // END_SUB_TUTORIAL

    // ADD ALL THE COLLISION OBJECTS/OBSTACLES HERE

    collision_objects[0].operation = collision_objects[0].ADD;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface arm("arm_with_torso");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");

    gripper.setPlanningTime(45.0);

    addCollisionObjects(planning_scene_interface);

    ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

    pick(arm);

    ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

    ros::shutdown();

    return 0;
}

// /*********************************************************************
//  * Software License Agreement (BSD License)
//  *
//  *  Copyright (c) 2013, SRI International
//  *  All rights reserved.
//  *
//  *  Redistribution and use in source and binary forms, with or without
//  *  modification, are permitted provided that the following conditions
//  *  are met:
//  *
//  *   * Redistributions of source code must retain the above copyright
//  *     notice, this list of conditions and the following disclaimer.
//  *   * Redistributions in binary form must reproduce the above
//  *     copyright notice, this list of conditions and the following
//  *     disclaimer in the documentation and/or other materials provided
//  *     with the distribution.
//  *   * Neither the name of SRI International nor the names of its
//  *     contributors may be used to endorse or promote products derived
//  *     from this software without specific prior written permission.
//  *
//  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  *  POSSIBILITY OF SUCH DAMAGE.
//  *********************************************************************/

// /* Author: Sachin Chitta */

// /* Modified by: Thomas Harrison */

// #include <ros/ros.h>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>

// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// // TF2
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <iostream>

// void openGripper(trajectory_msgs::JointTrajectory &posture)
// {
//     // BEGIN_SUB_TUTORIAL open_gripper
//     /* Add both finger joints of panda robot. */
//     posture.joint_names.resize(2);
//     posture.joint_names[0] = "r_gripper_finger_joint";
//     posture.joint_names[1] = "l_gripper_finger_joint";

//     /* Set them as open, wide enough for the object to fit. */
//     posture.points.resize(1);
//     posture.points[0].positions.resize(2);
//     posture.points[0].positions[0] = 0.04;
//     posture.points[0].positions[1] = 0.04;
//     posture.points[0].time_from_start = ros::Duration(0.5);
//     // END_SUB_TUTORIAL
// }

// void closedGripper(trajectory_msgs::JointTrajectory &posture)
// {
//     // BEGIN_SUB_TUTORIAL closed_gripper
//     /* Add both finger joints of panda robot. */
//     posture.joint_names.resize(2);
//     posture.joint_names[0] = "r_gripper_finger_joint";
//     posture.joint_names[1] = "l_gripper_finger_joint";

//     /* Set them as closed. */
//     posture.points.resize(1);
//     posture.points[0].positions.resize(2);
//     posture.points[0].positions[0] = 0.00;
//     posture.points[0].positions[1] = 0.00;
//     posture.points[0].time_from_start = ros::Duration(0.5);
//     // END_SUB_TUTORIAL
// }

// void pick(moveit::planning_interface::MoveGroupInterface &move_group)
// {
//     // BEGIN_SUB_TUTORIAL pick1
//     // Create a vector of grasps to be attempted, currently only creating single grasp.
//     // This is essentially useful when using a grasp generator to generate and test multiple grasps.
//     std::vector<moveit_msgs::Grasp> grasps;
//     grasps.resize(1);

//     // Setting grasp pose
//     // ++++++++++++++++++++++
//     // This is the pose of panda_link8. |br|
//     // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
//     // of the cube). |br|
//     // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
//     // extra padding)
//     grasps[0].grasp_pose.header.frame_id = "gripper";
//     tf2::Quaternion orientation;
//     orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
//     grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
//     grasps[0].grasp_pose.pose.position.x = 0.415;
//     grasps[0].grasp_pose.pose.position.y = 0;
//     grasps[0].grasp_pose.pose.position.z = 0.5;

//     // Setting pre-grasp approach
//     // ++++++++++++++++++++++++++
//     /* Defined with respect to frame_id */
//     grasps[0].pre_grasp_approach.direction.header.frame_id = "gripper";
//     /* Direction is set as positive x axis */
//     grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
//     grasps[0].pre_grasp_approach.min_distance = 0.095;
//     grasps[0].pre_grasp_approach.desired_distance = 0.115;

//     // Setting post-grasp retreat
//     // ++++++++++++++++++++++++++
//     /* Defined with respect to frame_id */
//     grasps[0].post_grasp_retreat.direction.header.frame_id = "gripper";
//     /* Direction is set as positive z axis */
//     grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
//     grasps[0].post_grasp_retreat.min_distance = 0.1;
//     grasps[0].post_grasp_retreat.desired_distance = 0.25;

//     // Setting posture of eef before grasp
//     // +++++++++++++++++++++++++++++++++++
//     openGripper(grasps[0].pre_grasp_posture);
//     // END_SUB_TUTORIAL

//     // BEGIN_SUB_TUTORIAL pick2
//     // Setting posture of eef during grasp
//     // +++++++++++++++++++++++++++++++++++
//     closedGripper(grasps[0].grasp_posture);
//     // END_SUB_TUTORIAL

//     // BEGIN_SUB_TUTORIAL pick3
//     // Set support surface as table1.
//     move_group.setSupportSurfaceName("table1");
//     // Call pick to pick up the object using the grasps given
//     move_group.pick("object", grasps);
//     // END_SUB_TUTORIAL
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "demo");
//     ros::NodeHandle node_handle;
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//     moveit::planning_interface::MoveGroupInterface arm("arm_with_torso");
//     moveit::planning_interface::MoveGroupInterface gripper("gripper");
//     gripper.setPlanningTime(45.0);

//     ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

//     pick(arm);

//     ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

//     //place(arm);

//     ros::shutdown();

//     return 0;
// }