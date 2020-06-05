#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>

#include <string>

std::string frame_id = "base_link"; //"base_link";
std::string frame_id_2 = "base_link"; //"base_link";


void openGripper(trajectory_msgs::JointTrajectory &posture)
{
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "r_gripper_finger_joint";
    posture.joint_names[1] = "l_gripper_finger_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(2);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
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
    posture.points.resize(2);
    posture.points[0].positions.resize(1);
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
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    grasps[0].grasp_pose.header.frame_id = frame_id;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.8;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

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
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    //move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
    // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface &group)
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
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* While placing it is the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = 0.8;
    place_location[0].place_pose.pose.position.y = 0.2;
    place_location[0].place_pose.pose.position.z = 1;

    // // Setting pre-place approach
    // // ++++++++++++++++++++++++++
    // /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = frame_id;
    // /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // // Setting post-grasp retreat
    // // ++++++++++++++++++++++++++
    // /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = frame_id;
    // /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    //group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    group.place("object", place_location);
    // END_SUB_TUTORIAL
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

    // // Add the first table where the cube will originally be kept.
    // collision_objects[0].id = "table1";
    // collision_objects[0].header.frame_id = "base_link";

    // /* Define the primitive and its dimensions. */
    // collision_objects[0].primitives.resize(1);
    // collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    // collision_objects[0].primitives[0].dimensions.resize(3);
    // collision_objects[0].primitives[0].dimensions[0] = 0.2;
    // collision_objects[0].primitives[0].dimensions[1] = 0.4;
    // collision_objects[0].primitives[0].dimensions[2] = 0.1;

    // /* Define the pose of the table. */
    // collision_objects[0].primitive_poses.resize(1);
    // collision_objects[0].primitive_poses[0].position.x = 0.8;
    // collision_objects[0].primitive_poses[0].position.y = 0;
    // collision_objects[0].primitive_poses[0].position.z = 0.95;
    // collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // // END_SUB_TUTORIAL

    // collision_objects[0].operation = collision_objects[0].ADD;

    // // BEGIN_SUB_TUTORIAL table2
    // // Add the second table where we will be placing the cube.
    // collision_objects[1].id = "table2";
    // collision_objects[1].header.frame_id = "base_link";

    // /* Define the primitive and its dimensions. */
    // collision_objects[1].primitives.resize(1);
    // collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    // collision_objects[1].primitives[0].dimensions.resize(3);
    // collision_objects[1].primitives[0].dimensions[0] = 0.4;
    // collision_objects[1].primitives[0].dimensions[1] = 0.2;
    // collision_objects[1].primitives[0].dimensions[2] = 0.4;

    // /* Define the pose of the table. */
    // collision_objects[1].primitive_poses.resize(1);
    // collision_objects[1].primitive_poses[0].position.x = 0.3;
    // collision_objects[1].primitive_poses[0].position.y = 0.5;
    // collision_objects[1].primitive_poses[0].position.z = 0.8;
    // collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // // END_SUB_TUTORIAL

    // collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[0].header.frame_id = frame_id_2;
    collision_objects[0].id = "object";
    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.02;
    collision_objects[0].primitives[0].dimensions[1] = 0.02;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.8;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.5;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(2.0).sleep(); //< Uses system time to delay for 1.0 second.

    // Define the attached object message
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // We will use this message to add or
    // subtract the object from the world
    // and to attach the object to the robot.
    moveit_msgs::AttachedCollisionObject attached_object;

    // Note that attaching an object to the robot requires
    // the corresponding operation to be specified as an ADD operation.
    attached_object.object.operation = attached_object.object.ADD;

    // Since we are attaching the object to the robot hand to simulate picking up the object,
    // we want the collision checker to ignore collisions between the object and the robot hand.
    attached_object.touch_links = std::vector<std::string>{"gripper", "r_gripper_finger_joint", "l_gripper_finger_joint","arm_with_torso"};

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface arm("arm_with_torso");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");

    // frame_id = gripper.getPlanningFrame();
    // ROS_INFO_STREAM("Reference frame: " << frame_id);

    gripper.setPlanningTime(45.0);
    arm.setPlanningTime(45.0);

    addCollisionObjects(planning_scene_interface);

    ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

    pick(arm);

    ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

    // place(arm);

    ros::shutdown();

    return 0;
}

// /*********************************************************************
//  * Software License Agreement (BSD License)
//  *
//  *  Copyright (c) 2012, Willow Garage, Inc.
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
//  *   * Neither the name of Willow Garage nor the names of its
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

// /* Author: Sachin Chitta, Michael Lautman */

// #include <ros/ros.h>
// #include <geometry_msgs/Pose.h>

// // MoveIt
// #include <moveit_msgs/PlanningScene.h>
// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/GetStateValidity.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/ApplyPlanningScene.h>

// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/robot_state/conversions.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "planning_scene_ros_api_tutorial");
//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   ros::NodeHandle node_handle;
//   // BEGIN_TUTORIAL
//   //
//   // Visualization
//   // ^^^^^^^^^^^^^
//   // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
//   // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
//   moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
//   visual_tools.deleteAllMarkers();

//   // ROS API
//   // ^^^^^^^
//   // The ROS API to the planning scene publisher is through a topic interface
//   // using "diffs". A planning scene diff is the difference between the current
//   // planning scene (maintained by the move_group node) and the new planning
//   // scene desired by the user.
//   //
//   // Advertise the required topic
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   // We create a publisher and wait for subscribers.
//   // Note that this topic may need to be remapped in the launch file.
//   ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
//   ros::WallDuration sleep_t(0.5);
//   while (planning_scene_diff_publisher.getNumSubscribers() < 1)
//   {
//     sleep_t.sleep();
//   }
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//   // Define the attached object message
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   // We will use this message to add or
//   // subtract the object from the world
//   // and to attach the object to the robot.
//   moveit_msgs::AttachedCollisionObject attached_object;
//   attached_object.link_name = "l_gripper_finger_link";
//   /* The header must contain a valid TF frame*/
//   attached_object.object.header.frame_id = "l_gripper_finger_link";
//   /* The id of the object */
//   attached_object.object.id = "box";

//   /* A default pose */
//   geometry_msgs::Pose pose;
//   pose.orientation.w = 1.0;

//   /* Define a box to be attached */
//   shape_msgs::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[0] = 0.08;
//   primitive.dimensions[1] = 0.08;
//   primitive.dimensions[2] = 0.08;

//   attached_object.object.primitives.push_back(primitive);
//   attached_object.object.primitive_poses.push_back(pose);

//   // Note that attaching an object to the robot requires
//   // the corresponding operation to be specified as an ADD operation.
//   attached_object.object.operation = attached_object.object.ADD;

//   // Since we are attaching the object to the robot hand to simulate picking up the object,
//   // we want the collision checker to ignore collisions between the object and the robot hand.
//   attached_object.touch_links = std::vector<std::string>{ "gripper_link", "l_gripper_finger_link", "r_gripper_finger_link" };

//   // Add an object into the environment
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   // Add the object into the environment by adding it to
//   // the set of collision objects in the "world" part of the
//   // planning scene. Note that we are using only the "object"
//   // field of the attached_object message here.
//   ROS_INFO("Adding the object into the world at the location of the hand.");
//   moveit_msgs::PlanningScene planning_scene;
//   planning_scene.world.collision_objects.push_back(attached_object.object);
//   planning_scene.is_diff = true;
//   planning_scene_diff_publisher.publish(planning_scene);
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Interlude: Synchronous vs Asynchronous updates
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   // There are two separate mechanisms available to interact
//   // with the move_group node using diffs:
//   //
//   // * Send a diff via a rosservice call and block until
//   //   the diff is applied (synchronous update)
//   // * Send a diff via a topic, continue even though the diff
//   //   might not be applied yet (asynchronous update)
//   //
//   // While most of this tutorial uses the latter mechanism (given the long sleeps
//   // inserted for visualization purposes asynchronous updates do not pose a problem),
//   // it would is perfectly justified to replace the planning_scene_diff_publisher
//   // by the following service client:
//   ros::ServiceClient planning_scene_diff_client =
//       node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
//   planning_scene_diff_client.waitForExistence();
//   // and send the diffs to the planning scene via a service call:
//   moveit_msgs::ApplyPlanningScene srv;
//   srv.request.scene = planning_scene;
//   planning_scene_diff_client.call(srv);
//   // Note that this does not continue until we are sure the diff has been applied.
//   //
//   // Attach an object to the robot
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   // When the robot picks up an object from the environment, we need to
//   // "attach" the object to the robot so that any component dealing with
//   // the robot model knows to account for the attached object, e.g. for
//   // collision checking.
//   //
//   // Attaching an object requires two operations
//   //  * Removing the original object from the environment
//   //  * Attaching the object to the robot

//   /* First, define the REMOVE object message*/
//   moveit_msgs::CollisionObject remove_object;
//   remove_object.id = "box";
//   remove_object.header.frame_id = "base_link";
//   remove_object.operation = remove_object.REMOVE;

//   // Note how we make sure that the diff message contains no other
//   // attached objects or collisions objects by clearing those fields
//   // first.
//   /* Carry out the REMOVE + ATTACH operation */
//   ROS_INFO("Attaching the object to the hand and removing it from the world.");
//   planning_scene.world.collision_objects.clear();
//   planning_scene.world.collision_objects.push_back(remove_object);
//   planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
//   planning_scene_diff_publisher.publish(planning_scene);

//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Detach an object from the robot
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   // Detaching an object from the robot requires two operations
//   //  * Detaching the object from the robot
//   //  * Re-introducing the object into the environment

//   /* First, define the DETACH object message*/
//   moveit_msgs::AttachedCollisionObject detach_object;
//   detach_object.object.id = "box";
//   detach_object.link_name = "gripper_link";
//   detach_object.object.operation = attached_object.object.REMOVE;

//   // Note how we make sure that the diff message contains no other
//   // attached objects or collisions objects by clearing those fields
//   // first.
//   /* Carry out the DETACH + ADD operation */
//   ROS_INFO("Detaching the object from the robot and returning it to the world.");
//   planning_scene.robot_state.attached_collision_objects.clear();
//   planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
//   planning_scene.robot_state.is_diff = true;
//   planning_scene.world.collision_objects.clear();
//   planning_scene.world.collision_objects.push_back(attached_object.object);
//   planning_scene.is_diff = true;
//   planning_scene_diff_publisher.publish(planning_scene);

//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Remove the object from the collision world
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   // Removing the object from the collision world just requires
//   // using the remove object message defined earlier.
//   // Note, also how we make sure that the diff message contains no other
//   // attached objects or collisions objects by clearing those fields
//   // first.
//   ROS_INFO("Removing the object from the world.");
//   planning_scene.robot_state.attached_collision_objects.clear();
//   planning_scene.world.collision_objects.clear();
//   planning_scene.world.collision_objects.push_back(remove_object);
//   planning_scene_diff_publisher.publish(planning_scene);
//   // END_TUTORIAL

//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");

//   ros::shutdown();
//   return 0;
// }

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

// #include <string>

// std::string frame_id = "arm"; //"base_link";

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
//     posture.points[0].positions[0] = 0.045; // open gripper position (50mm in each direction)
//     posture.points[0].positions[1] = 0.045;
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

//     // DEFINE POSE OF THE FETCH ROBOT ARM

//     ROS_INFO_STREAM("Pick Reference frame: " << move_group.getPlanningFrame());

//     grasps[0].grasp_pose.header.frame_id = frame_id;
//     tf2::Quaternion orientation;
//     orientation.setRPY(0, 0, 0);
//     grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
//     grasps[0].grasp_pose.pose.position.x = 0.6;
//     grasps[0].grasp_pose.pose.position.y = 0.0;
//     grasps[0].grasp_pose.pose.position.z = 0.4;

//     // Setting pre-grasp approach
//     // ++++++++++++++++++++++++++
//     /* Defined with respect to frame_id */
//     grasps[0].pre_grasp_approach.direction.header.frame_id = frame_id;
//     /* Direction is set as positive x axis */
//     grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
//     grasps[0].pre_grasp_approach.min_distance = 0.095;
//     grasps[0].pre_grasp_approach.desired_distance = 0.115;

//     // Setting post-grasp retreat
//     // ++++++++++++++++++++++++++
//     /* Defined with respect to frame_id */
//     grasps[0].post_grasp_retreat.direction.header.frame_id = frame_id;
//     /* Direction is set as positive z axis */
//     grasps[0].post_grasp_retreat.direction.vector.x = -0.5;
//     grasps[0].post_grasp_retreat.min_distance = 0.1;
//     grasps[0].post_grasp_retreat.desired_distance = 0.25;

//     // Setting posture of eef before grasp
//     // +++++++++++++++++++++++++++++++++++
//     openGripper(grasps[0].pre_grasp_posture);
//     // // END_SUB_TUTORIAL
//     // move_group.move();

//     // moveit_msgs::MoveGroupGoal goal.

//     // // BEGIN_SUB_TUTORIAL pick2
//     // // Setting posture of eef during grasp
//     // // +++++++++++++++++++++++++++++++++++
//     closedGripper(grasps[0].grasp_posture);

//     // BEGIN_SUB_TUTORIAL pick3
//     // Set support surface as table1.
//     //move_group.setSupportSurfaceName("table1");
//     // Call pick to pick up the object using the grasps given
//     move_group.pick("bottle", grasps);
//     //END_SUB_TUTORIAL
// }

// void place(moveit::planning_interface::MoveGroupInterface &group)
// {
//     // BEGIN_SUB_TUTORIAL place
//     // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
//     // location in
//     // verbose mode." This is a known issue and we are working on fixing it. |br|
//     // Create a vector of placings to be attempted, currently only creating single place location.
//     std::vector<moveit_msgs::PlaceLocation> place_location;
//     place_location.resize(1);

//     // Setting place location pose
//     // +++++++++++++++++++++++++++
//     place_location[0].place_pose.header.frame_id = frame_id;
//     tf2::Quaternion orientation;
//     orientation.setRPY(0, 0, 0);
//     place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

//     /* While placing it is the exact location of the center of the object. */
//     place_location[0].place_pose.pose.position.x = 0.6;
//     place_location[0].place_pose.pose.position.y = 0.1;
//     place_location[0].place_pose.pose.position.z = 1.166;

//     // Setting pre-place approach
//     // ++++++++++++++++++++++++++
//     /* Defined with respect to frame_id */
//     place_location[0].pre_place_approach.direction.header.frame_id = frame_id;
//     /* Direction is set as negative z axis */
//     place_location[0].pre_place_approach.direction.vector.z = -1.0;
//     place_location[0].pre_place_approach.min_distance = 0.095;
//     place_location[0].pre_place_approach.desired_distance = 0.115;

//     // Setting post-grasp retreat
//     // ++++++++++++++++++++++++++
//     /* Defined with respect to frame_id */
//     place_location[0].post_place_retreat.direction.header.frame_id = frame_id;
//     /* Direction is set as negative y axis */
//     place_location[0].post_place_retreat.direction.vector.y = -1.0;
//     place_location[0].post_place_retreat.min_distance = 0.1;
//     place_location[0].post_place_retreat.desired_distance = 0.25;

//     //closedGripper(place_location[0].post_place_posture);

//     // Setting posture of eef after placing object
//     // +++++++++++++++++++++++++++++++++++++++++++
//     /* Similar to the pick case */
//     openGripper(place_location[0].post_place_posture);

//     // Set support surface as table2.
//     // group.setSupportSurfaceName("shelf");
//     // Call place to place the object using the place locations given.
//     group.place("bottle", place_location);
//     // END_SUB_TUTORIAL
// }

// void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
// {
//     // BEGIN_SUB_TUTORIAL table1
//     //
//     // Creating Environment
//     // ^^^^^^^^^^^^^^^^^^^^
//     // Create vector to hold 3 collision objects.
//     std::vector<moveit_msgs::CollisionObject> collision_objects;
//     collision_objects.resize(1);

//     // BEGIN_SUB_TUTORIAL object
//     // Define the object that we will be manipulating
//     collision_objects[0].header.frame_id = "base_link";
//     collision_objects[0].id = "bottle";

//     /* Define the primitive and its dimensions. */
//     collision_objects[0].primitives.resize(1);
//     collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
//     collision_objects[0].primitives[0].dimensions.resize(3);

//     collision_objects[0].primitives[0].dimensions[0] = 0.04;
//     collision_objects[0].primitives[0].dimensions[1] = 0.04;
//     collision_objects[0].primitives[0].dimensions[2] = 0.04;
//     // collision_objects[0].primitives[0].dimensions[0] = 0.07;
//     // collision_objects[0].primitives[0].dimensions[1] = 0.07;
//     // collision_objects[0].primitives[0].dimensions[2] = 0.2;
//     collision_objects[0].operation = collision_objects[0].ADD;

//     /* Define the pose of the object. */
//     collision_objects[0].primitive_poses.resize(1);
//     collision_objects[0].primitive_poses[0].position.x = 0.6;
//     collision_objects[0].primitive_poses[0].position.y = 0.0;
//     collision_objects[0].primitive_poses[0].position.z = 0.38;
//     // END_SUB_TUTORIAL

//     // BEGIN_SUB_TUTORIAL object
//     // Define the object that we will be manipulating
//     // collision_objects[1].header.frame_id = "arm";
//     // collision_objects[1].id = "shelf";

//     // /* Define the primitive and its dimensions. */
//     // collision_objects[1].primitives.resize(1);
//     // collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
//     // collision_objects[1].primitives[0].dimensions.resize(3);
//     // collision_objects[1].primitives[0].dimensions[0] = 0.5;
//     // collision_objects[1].primitives[0].dimensions[1] = 5;
//     // collision_objects[1].primitives[0].dimensions[2] = 0.06;
//     // collision_objects[1].operation = collision_objects[0].ADD;

//     // /* Define the pose of the object. */
//     // collision_objects[1].primitive_poses.resize(1);
//     // collision_objects[1].primitive_poses[0].position.x = 1.1;
//     // collision_objects[1].primitive_poses[0].position.y = 0;
//     // collision_objects[1].primitive_poses[0].position.z = 1.03;
//     // END_SUB_TUTORIAL

//     // ADD ALL THE COLLISION OBJECTS/OBSTACLES HERE

//     planning_scene_interface.applyCollisionObjects(collision_objects);
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "demo");
//     ros::NodeHandle node_handle;
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     ros::WallDuration(2.0).sleep(); //< Uses system time to delay for 1.0 second.

//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//     moveit::planning_interface::MoveGroupInterface arm("arm_with_torso");
//     moveit::planning_interface::MoveGroupInterface gripper("gripper");

//     // frame_id = arm.getPlanningFrame();
//     // ROS_INFO_STREAM("Reference frame: " << frame_id);

//     gripper.setPlanningTime(45.0);
//     arm.setPlanningTime(45.0);

//     addCollisionObjects(planning_scene_interface);

//     ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

//     pick(arm);

//     ros::WallDuration(1.0).sleep(); //< Uses system time to delay for 1.0 second.

//     // place(arm);

//     ros::shutdown();

//     return 0;
// }