#include "vision_c.h"

Vision::Vision(ros::NodeHandle nh) : nh_(nh)
{
    sub_ = nh_.subscribe("drink_selection", 10, &Vision::drinkSelectionCallback, this);
    pub_ = nh_.advertise<geometry_msgs::PoseArray>("vision_poses", 1000);

    std::random_device rd;
    nd_generator = new std::default_random_engine(rd()); // seed the generator
    random_double = new std::normal_distribution<double>(-1.0, 1.0);
}

void Vision::drinkSelectionCallback(const drink_menu::drink::ConstPtr &msg)
{
    ROS_INFO_STREAM("Detecting drink: " << msg->drink);
    auto ingredients = msg->ingredients;
    std::vector<geometry_msgs::Pose> poses;
    for (auto ingredient : ingredients)
    {
        poses.push_back(imageProcessing(ingredient));
    }

    geometry_msgs::PoseArray poseArray;
    poseArray.poses = poses;

    publishVisionPoses(poseArray);
}

void Vision::publishVisionPoses(const geometry_msgs::PoseArray &poses)
{
    pub_.publish(poses);
}

geometry_msgs::Pose Vision::imageProcessing(const std::string drink)
{
    geometry_msgs::Pose pose;
    // pose.position.x = (*random_double)(*nd_generator);
    // pose.position.y = (*random_double)(*nd_generator);
    // pose.position.z = (*random_double)(*nd_generator);
    pose.position.x = 1.05; //0.9
    pose.position.y = 0.10; //0.1
    pose.position.z = 1.05;

    return pose;
}

void Vision::seperateThread()
{
    ros::Rate rate_limiter(1.0);
    while (ros::ok())
    {
    }
}