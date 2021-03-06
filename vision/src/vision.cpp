

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "std_msgs/String.h"
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_datatypes.h>
#include <string>
//#include <tf2_geometry_msgs.h>

//this is the code to see the tthough the camera

//000tatic const std::string OPENCV_WINDOW = "Image window";

//tf::TransformListener listener;
/*
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

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};
*/
class ar_finder
{

public:
    ros::NodeHandle vs_;
    ros::Subscriber see_;
    geometry_msgs::Pose pose_vodka;
    geometry_msgs::Pose pose_vermouth;
    geometry_msgs::Pose pose_gin;
    geometry_msgs::Pose pose_campari;

    ar_finder(ros::NodeHandle vs_);

    geometry_msgs::Pose get_drink(const std::string drink);
    void ar_finder_Callback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &mag);
};

ar_finder::ar_finder(ros::NodeHandle vs) : vs_(vs)
{
    see_ = vs_.subscribe("drink_selection", 1, &ar_finder::ar_finder_Callback, this);
}

void ar_finder::ar_finder_Callback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &mag)
{
    geometry_msgs::Pose pose;
    int32_t drink;
    drink = mag->markers.at(0).id;
    switch (drink)
    {
    case '1':
        ROS_INFO_STREAM("vermouth");
        if (pose_vermouth.position.x == 0 & pose_vermouth.position.z == 0 & pose_vermouth.position.y == 0)
        {
            pose_vermouth.position.x = mag->markers.at(0).pose.pose.position.x;
            pose_vermouth.position.z = mag->markers.at(0).pose.pose.orientation.z;
            pose_vermouth.position.y = mag->markers.at(0).pose.pose.orientation.y;
        }
        ROS_INFO_STREAM("vermouth postion ... (x,y,z");
        ROS_INFO_STREAM(pose_vermouth.position.x);
        ROS_INFO_STREAM(pose_vermouth.position.y);
        ROS_INFO_STREAM(pose_vermouth.position.z);
        break;
    case '2':
        ROS_INFO_STREAM("campari");
        if (pose_campari.position.x == 0 & pose_campari.position.z == 0 & pose_campari.position.y == 0)
        {
            pose_campari.position.x = mag->markers.at(0).pose.pose.position.x;
            pose_campari.position.z = mag->markers.at(0).pose.pose.orientation.z;
            pose_campari.position.y = mag->markers.at(0).pose.pose.orientation.y;
        }
        ROS_INFO_STREAM("campari postion ... (x,y,z");
        ROS_INFO_STREAM(pose_campari.position.x);
        ROS_INFO_STREAM(pose_campari.position.y);
        ROS_INFO_STREAM(pose_campari.position.z);
        break;
    case '3':
        ROS_INFO_STREAM("gin");
        if (pose_gin.position.x == 0 & pose_gin.position.z == 0 & pose_gin.position.y == 0)
        {
            pose_gin.position.x = mag->markers.at(0).pose.pose.position.x;
            pose_gin.position.z = mag->markers.at(0).pose.pose.orientation.z;
            pose_gin.position.y = mag->markers.at(0).pose.pose.orientation.y;
        }
        ROS_INFO_STREAM("gin postion ... (x,y,z");
        ROS_INFO_STREAM(pose_gin.position.x);
        ROS_INFO_STREAM(pose_gin.position.y);
        ROS_INFO_STREAM(pose_gin.position.z);

        break;

    case '4':
        ROS_INFO_STREAM("vodka");
        if (pose_vodka.position.x == 0 & pose_vodka.position.z == 0 & pose_vodka.position.y == 0)
        {
            pose_vodka.position.x = mag->markers.at(0).pose.pose.position.x;
            pose_vodka.position.z = mag->markers.at(0).pose.pose.orientation.z;
            pose_vodka.position.y = mag->markers.at(0).pose.pose.orientation.y;
        }
        ROS_INFO_STREAM("vodka postion ... (x,y,z");
        ROS_INFO_STREAM(pose_vodka.position.x);
        ROS_INFO_STREAM(pose_vodka.position.y);
        ROS_INFO_STREAM(pose_vodka.position.z);
        break;

    default:
        ROS_INFO_STREAM("can't find any drinks");
    }
}

geometry_msgs::Pose ar_finder::get_drink(const std::string drink)
{
    std::string vermouth;
    std::string campari;
    std::string gin;
    std::string vodka;

    geometry_msgs::Pose pose;

    if (drink == vermouth)
    {
        pose = pose_vermouth;
        return pose;
    }

    else if (drink == campari)
    {
        pose = pose_campari;
        return pose;
    }
    else if (drink == gin)
    {
        pose = pose_gin;
        return pose;
    }

    else if (drink == vodka)
    {
        pose = pose_campari;
        return pose;
    }

    else
        ROS_INFO_STREAM("can't find any drinks");
}

//this is the draft for the code to get the codrdiante from the head camera link to the base link
/*
  tf::Transform cam_to_target;
  tf::poseMsgToTF(position_->pose.pose, cam_to_target);

  tf::StampedTransform req_to_cam;
  listener_.lookupTransform(req.base_frame, position_->header.frame_id, ros::Time(0), req_to_cam);

  tf::Transform req_to_target;
  req_to_target = req_to_cam * cam_to_target;

  tf::poseTFToMsg(req_to_target, res.pose);
  return true;
 

*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_pose_marker");
    ros::NodeHandle vs2;

    ar_finder ar(vs2);
    ros::spin();
    return 0;
}
