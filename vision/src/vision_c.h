#ifndef VISION_H
#define VISION_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drink_menu/drink.h"
#include "geometry_msgs/PoseArray.h"
#include <random>

class Vision
{
public:
    Vision(ros::NodeHandle nh);

    void seperateThread();

private:
    void drinkSelectionCallback(const drink_menu::drink::ConstPtr &msg);
    void publishVisionPoses(const geometry_msgs::PoseArray &poses);
    geometry_msgs::Pose imageProcessing(const std::string drink);

public:
    ros::NodeHandle nh_;

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::default_random_engine *nd_generator;
    std::normal_distribution<double> *random_double;
};

#endif //VISION_H