
#ifndef INTEGRATION_H
#define INTEGRATION_H

#include "ros/ros.h"
#include <vector>
#include <deque>
#include "geometry_msgs/PoseArray.h"
#include "grasping/move.h"

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    ros::ServiceClient client_shelf_to_bar_;
    ros::ServiceClient client_bar_to_shelf_;

    geometry_msgs::Pose surface_;
    geometry_msgs::Point bottle_offset_;
};

#endif //INTEGRATION_H