#include "grasp.h"

const std::string Grasp::FRAME_ID = "arm";

void Grasp::openGripper(trajectory_msgs::JointTrajectory &posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "r_gripper_finger_joint";
    posture.joint_names[1] = "l_gripper_finger_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.045; // open gripper position (50mm in each direction)
    posture.points[0].positions[1] = 0.045;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void Grasp::closeGripper(trajectory_msgs::JointTrajectory &posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "r_gripper_finger_joint";
    posture.joint_names[1] = "l_gripper_finger_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void Grasp::pick(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose pose)
{
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // DEFINE POSE OF THE FETCH ROBOT ARM

    grasps[0].grasp_pose.header.frame_id = FRAME_ID;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position = pose;

    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = FRAME_ID;
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = FRAME_ID;
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.x = -0.5;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);

    closeGripper(grasps[0].grasp_posture);

    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
}

void moveObject(geometry_msgs::Pose current, geometry_msgs::Pose target)
{
}

void createObject();