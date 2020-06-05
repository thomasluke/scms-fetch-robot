#include "grasp.h"

const std::string Grasp::FRAME_ID = "gripper";

Grasp::Grasp(ros::NodeHandle &nh, std::string move_group)
    : nh_(nh), move_group_(move_group)
{
}

void Grasp::openGripper(trajectory_msgs::JointTrajectory &posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "r_gripper_finger_joint";
    posture.joint_names[1] = "l_gripper_finger_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.045; // open gripper position (50mm in each direction)
    posture.points[0].positions[1] = 0.045;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void Grasp::closeGripper(trajectory_msgs::JointTrajectory &posture)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "r_gripper_finger_joint";
    posture.joint_names[1] = "l_gripper_finger_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void Grasp::pick(moveit::planning_interface::MoveGroupInterface &move_group,
                 const std::string &name,
                 const geometry_msgs::Pose &object)
{
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // DEFINE POSE OF THE FETCH ROBOT ARM
    grasps[0].grasp_pose.header.frame_id = FRAME_ID;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    grasps[0].grasp_pose.pose = object;
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    addPoints(grasps[0].grasp_pose.pose.position, gripper_offset_); //< offset the gripper position relative to the object.

    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = FRAME_ID;
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = FRAME_ID;
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.x = -0.5;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);

    closeGripper(grasps[0].grasp_posture);

    // Call pick to pick up the object using the grasps given
    move_group.pick(name, grasps);
}

void Grasp::place(moveit::planning_interface::MoveGroupInterface &move_group,
                  const std::string &name,
                  const geometry_msgs::Pose &object)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    place_location[0].place_pose.header.frame_id = FRAME_ID;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    place_location[0].place_pose.pose = object;
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    addPoints(place_location[0].place_pose.pose.position, gripper_offset_); //< offset the gripper position relative to the object.

    place_location[0].pre_place_approach.direction.header.frame_id = FRAME_ID;
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    place_location[0].post_place_retreat.direction.header.frame_id = FRAME_ID;
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    //move_group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    move_group.place(name, place_location);
}

void addBottleObject(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                     const std::string &name,
                     const geometry_msgs::Pose &bottle)
{
    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = "base_link";
    collision_object.id = name;

    /* Define the primitive and its dimensions. */
    collision_object.primitives.resize(1);
    collision_object.primitives[0].type = collision_object.primitives[0].BOX;
    collision_object.primitives[0].dimensions.resize(3);
    collision_object.primitives[0].dimensions[0] = 0.07;
    collision_object.primitives[0].dimensions[1] = 0.07;
    collision_object.primitives[0].dimensions[2] = 0.20;

    /* Define the pose of the object. */
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0] = bottle;
    collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObject(collision_object);
}

void Grasp::setupScene(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    //Add Shelf
    {
        // Add the shelf where the cube will originally be kept.
        moveit_msgs::CollisionObject collision_object;
        collision_object.id = "shelf";
        collision_object.header.frame_id = "base_link";

        /* Define the primitive and its dimensions. */
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[0] = 0.2;
        collision_object.primitives[0].dimensions[1] = 0.4;
        collision_object.primitives[0].dimensions[2] = 0.1;

        /* Define the pose of the table. */
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position.x = 0.8;
        collision_object.primitive_poses[0].position.y = 0;
        collision_object.primitive_poses[0].position.z = 0.95;
        collision_object.primitive_poses[0].orientation.w = 1.0;
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }

    //Add Bar
    {
        // Add the bar table where we will be placing the cube.
        moveit_msgs::CollisionObject collision_object;
        collision_object.id = "bar";
        collision_object.header.frame_id = "base_link";

        /* Define the primitive and its dimensions. */
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[0] = 0.4;
        collision_object.primitives[0].dimensions[1] = 0.2;
        collision_object.primitives[0].dimensions[2] = 0.4;

        /* Define the pose of the table. */
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position.x = 0.3;
        collision_object.primitive_poses[0].position.y = 0.5;
        collision_object.primitive_poses[0].position.z = 0.4;
        collision_object.primitive_poses[0].orientation.w = 1.0;
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void moveObject(geometry_msgs::Pose current, geometry_msgs::Pose target)
{
}

void Grasp::setGripperOffset(const geometry_msgs::Point &offset)
{
    gripper_offset_ = offset;
}

void Grasp::addPoints(geometry_msgs::Point &pt1, const geometry_msgs::Point &pt2)
{
    pt1.x += pt2.x;
    pt1.y += pt2.y;
    pt1.z += pt2.z;
}