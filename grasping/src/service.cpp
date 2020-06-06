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

// void printV(tf2::Vector3 v)
// {
//     ROS_INFO_STREAM("Vector: (" << v.getX() << ", " << v.getY() << ", " << v.getZ() << ")");
// }

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

    // tf2::Quaternion o1;
    // o1.setRPY(0, 0, -(M_PI / 2));
    // tf2::Quaternion o2;
    // o2.setRPY(0, 0, 0);
    // printV(grasp->quaternionToVector(tf2::toMsg(o1)));
    // printV(grasp->quaternionToVector(tf2::toMsg(o2)));
    // printV(grasp->quaternionToVector(tf2::toMsg((o2 + o1))));

    // return 0;

    geometry_msgs::Point gripper_offset;
    gripper_offset.x = 0;
    gripper_offset.y = 0;
    gripper_offset.z = 0.1;
    grasp->setGripperOffset(gripper_offset);

    geometry_msgs::Point pick_shelf_offset;
    pick_shelf_offset.x = -0.2;
    pick_shelf_offset.y = 0.0;
    pick_shelf_offset.z = 0.0;
    geometry_msgs::Point pick_bar_offset;
    pick_bar_offset.x = 0.0;
    pick_bar_offset.y = -0.2;
    pick_bar_offset.z = 0.0;
    grasp->setPickOffset(pick_shelf_offset, pick_bar_offset);

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

    return 0;
}