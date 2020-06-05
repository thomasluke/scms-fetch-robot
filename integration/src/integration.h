
#ifndef INTEGRATION_H
#define INTEGRATION_H

#include "ros/ros.h"
#include <vector>
#include <deque>
#include "geometry_msgs/PoseArray.h"
#include "grasping/move.h"

class Integration
{
public:
    Integration(ros::NodeHandle nh);
    ~Integration();

    void setSurface(const geometry_msgs::Pose &surface);
    void setBottleOffset(const geometry_msgs::Point &offset);

private:
    void visionCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
    void addPoints(geometry_msgs::Point &pt1, const geometry_msgs::Point &pt2);

public:
    ros::NodeHandle nh_;

private:
    ros::Subscriber sub_;
    ros::ServiceClient client_;

    geometry_msgs::Pose surface_;
    geometry_msgs::Point bottle_offset_;
};

#endif //INTEGRATION_H