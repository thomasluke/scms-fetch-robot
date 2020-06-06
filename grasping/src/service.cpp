#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grasping/move.h"
#include "grasp.h"
#include <string>
#include <thread>

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
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::shared_ptr<Grasp> grasp(new Grasp(n, "arm_with_torso"));
    std::thread t(&Grasp::seperateThread, grasp);

    geometry_msgs::Point gripper_offset;
    gripper_offset.x = 0;
    gripper_offset.y = 0;
    gripper_offset.z = 0.1;
    grasp->setGripperOffset(gripper_offset);

    geometry_msgs::Point pick_offset;
    pick_offset.x = -0.2;
    pick_offset.y = 0;
    pick_offset.z = 0.0;
    grasp->setPickOffset(pick_offset);

    geometry_msgs::Point bottle_offset;
    bottle_offset.x = 0;
    bottle_offset.y = 0;
    bottle_offset.z = 0.1;
    grasp->setBottleOffset(bottle_offset);

    geometry_msgs::Point fetch_offset;
    fetch_offset.x = -(0.6);
    fetch_offset.y = -(1.5);
    fetch_offset.z = -(0.0);
    grasp->setFetchOffset(fetch_offset);

    grasp->setupScene();

    geometry_msgs::Pose current;
    current.position.x = 0.5;
    current.position.y = 0.3;
    current.position.z = 0.8;
    geometry_msgs::Pose target;
    target.position.x = 0.4;
    target.position.y = -0.4;
    target.position.z = 0.5;

    grasping::move move;
    move.request.current = current;
    move.request.target = target;

    // grasp->graspingCallback(move.request, move.response);

    ros::waitForShutdown();
    t.join();

    return 0;
}