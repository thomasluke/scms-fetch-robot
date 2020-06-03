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

class Grasp
{
public:
    static const std::string FRAME_ID;

    void pick(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose pose);

    void openGripper(trajectory_msgs::JointTrajectory &posture);
    void closeGripper(trajectory_msgs::JointTrajectory &posture);
};
