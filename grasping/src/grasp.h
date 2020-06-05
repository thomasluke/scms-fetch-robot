#ifndef GRASP_H
#define GRASP_H

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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <string>

class Grasp
{
public:
    static const std::string FRAME_ID;

    Grasp(ros::NodeHandle &nh, std::string move_group);

    void pick(const std::string &name, const geometry_msgs::Pose &object);
    void place(const std::string &name, const geometry_msgs::Pose &object);

    void setGripperOffset(const geometry_msgs::Point &offset);
    void setupScene();
    void moveBottle(geometry_msgs::Pose current, geometry_msgs::Pose target);

private:
    void openGripper(trajectory_msgs::JointTrajectory &posture);
    void closeGripper(trajectory_msgs::JointTrajectory &posture);
    void addPoints(geometry_msgs::Point &pt1, const geometry_msgs::Point &pt2);
    void addBottleObject(const std::string &name, const geometry_msgs::Pose &bottle);
    void removeBottleObject(const std::string &name);

public:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_;

private:
    geometry_msgs::Point gripper_offset_;
};

#endif // GRASP_H