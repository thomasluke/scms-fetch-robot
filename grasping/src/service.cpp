#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grasping/pose.h"

#include "grasp.h"

bool moveToPose(grasping::pose::Request &req, grasping::pose::Response &res)
{
    res.success = true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gripper");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    ros::spin();
    ros::shutdown();
}