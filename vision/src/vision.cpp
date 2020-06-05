
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "std_msgs/String.h"
//#include ""ar_track_alvar/Alvar.h"
//#include "ar_track_alvar/MultiMarker.h"
//#include "drink_menu/drink.h"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh25_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh25_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/head_camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
/*
//////////////////////////////////////////
#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <tf/tf.h>  
#include <tf/transform_datatypes.h>

#include <tf/transform_listener.h>


void printPose(const ar_track_alvar_msgs::AlvarMarker::ConstPtr& msg)
{   
    tf::Pose marker_pose_in_camera_;    
    ar_track_alvar_msgs.framid 

    marker_pose_in_camera_.setOrigin(tf::Vector3(msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             msg.pose.pose.position.z));

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_subscriber");

    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("ar_pose_marker", 1000, printPose);

    ros::spin();

    return 0;

}

*/