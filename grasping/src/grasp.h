#ifndef GRASP_H
#define GRASP_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "grasping/move.h"

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

    bool pick(const std::string &name, const geometry_msgs::Pose &object);
    bool pickBar(const std::string &name, const geometry_msgs::Pose &object);
    bool pickShelf(const std::string &name, const geometry_msgs::Pose &object);
    bool place(const std::string &name, const geometry_msgs::Pose &object);
    bool placeBar(const std::string &name, const geometry_msgs::Pose &object);
    bool placeShelf(const std::string &name, const geometry_msgs::Pose &object);

    void setGripperOffset(const geometry_msgs::Point &offset);
    void setPickOffset(const geometry_msgs::Point &shelf_offset, const geometry_msgs::Point &bar_offset);
    void setFetchOffset(const geometry_msgs::Point &offset);
    void setBottleOffset(const geometry_msgs::Point &offset);
    void setupScene();
    bool moveBottle(geometry_msgs::Pose &current, geometry_msgs::Pose &target);
    bool moveBottleToShelf(geometry_msgs::Pose &current, geometry_msgs::Pose &target);
    bool moveBottleToBar(geometry_msgs::Pose &current, geometry_msgs::Pose &target);

    bool graspingCallback(grasping::move::Request &req, grasping::move::Response &res);
    bool shelfToBarCallback(grasping::move::Request &req, grasping::move::Response &res);
    bool barToShelfCallback(grasping::move::Request &req, grasping::move::Response &res);
    tf2::Vector3 quaternionToVector(const geometry_msgs::Quaternion &quaternion);
    void addQuaternion(geometry_msgs::Quaternion &qu1, const geometry_msgs::Quaternion &qu2);

private:
    void openGripper(trajectory_msgs::JointTrajectory &posture);
    void closeGripper(trajectory_msgs::JointTrajectory &posture);

    void addPoints(geometry_msgs::Point &pt1, const geometry_msgs::Point &pt2);

    void addBottleObject(const std::string &name, const geometry_msgs::Pose &bottle);
    void removeBottleObject(const std::string &name);

    bool moveItError(const moveit::planning_interface::MoveItErrorCode &ec);

public:
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;

    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_;

private:
    geometry_msgs::Point gripper_offset_;
    geometry_msgs::Point pick_shelf_offset_;
    geometry_msgs::Point pick_bar_offset_;
    geometry_msgs::Point bottle_offset_;
    geometry_msgs::Point fetch_offset_;
    ros::ServiceServer service1_;
    ros::ServiceServer service2_;
    ros::ServiceServer service3_;
};

#endif // GRASP_H