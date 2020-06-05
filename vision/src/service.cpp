#include "ros/ros.h"
#include "vision_c.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle n;

    std::shared_ptr<Vision> vision(new Vision(n));

    ros::spin();
    ros::shutdown();
}