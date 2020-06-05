#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grasping/move.h"
#include "grasp.h"
#include <string>

bool graspingCallback(grasping::move::Request &req, grasping::move::Response &res)
{
    auto s_pose = req.current.position;
    auto e_pose = req.target.position;

    ROS_INFO_STREAM("Moving from pose: (" << s_pose.x << ", " << s_pose.y << ", " << s_pose.z << ")");
    ROS_INFO_STREAM("To pose: (" << e_pose.x << ", " << e_pose.y << ", " << e_pose.z << ")\n");

    res.success = true;
    return true;
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

    ros::ServiceServer service = n.advertiseService("grasping_service", graspingCallback);

    Grasp grasp(n, "arm_with_torso");

    geometry_msgs::Point offset;
    offset.x = -0.2;
    offset.y = 0;
    offset.z = 0;
    grasp.setGripperOffset(offset);

    grasp.setupScene();

    

    ros::spin();
    ros::requestShutdown();
}