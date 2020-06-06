#include "ros/ros.h"
#include <thread>
#include "integration.h"

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void visionCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    ROS_INFO_STREAM("Vision callback Main");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "integration");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */

    ros::NodeHandle n;

    // Integration integration(n);
    std::shared_ptr<Integration> integration(new Integration(n));

    geometry_msgs::Point offset;
    offset.x = 0;
    offset.y = 0.08;
    offset.z = 0;
    integration->setBottleOffset(offset);

    geometry_msgs::Pose surface;
    surface.position.x = 0.50;
    surface.position.y = 0.75;
    surface.position.z = 1.025;

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, -(M_PI / 2));
    surface.orientation = tf2::toMsg(orientation);

    integration->setSurface(surface);

    ros::spin();
    ros::shutdown();

    return 0;
}