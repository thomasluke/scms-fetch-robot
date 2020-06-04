#include "ros/ros.h"
#include "integration.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "integration");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    Integration integration(n);

    ros::spin();
    ros::shutdown();

    return 0;
}
