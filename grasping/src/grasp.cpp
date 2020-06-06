#include "grasp.h"

const std::string Grasp::FRAME_ID = "base_link";

Grasp::Grasp(ros::NodeHandle &nh, std::string move_group)
    : nh_(nh), move_group_(move_group), spinner_(4)
{
    spinner_.start();
    service1_ = nh_.advertiseService("grasping_service", &Grasp::graspingCallback, this);
    service2_ = nh_.advertiseService("grasping_service_shelf_to_bar", &Grasp::shelfToBarCallback, this);
    service3_ = nh_.advertiseService("grasping_service_bar_to_shelf", &Grasp::barToShelfCallback, this);
    move_group_.setPlanningTime(45.0);
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

bool Grasp::pick(const std::string &name, const geometry_msgs::Pose &object)
{
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // DEFINE POSE OF THE FETCH ROBOT ARM
    grasps[0].grasp_pose.header.frame_id = FRAME_ID;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, (M_PI / 2));
    grasps[0].grasp_pose.pose = object;
    // addQuaternion(grasps[0].grasp_pose.pose.orientation, tf2::toMsg(orientation));
    // addPoints(grasps[0].grasp_pose.pose.position, pick_offset_);    //< offset the gripper position relative to the object.
    addPoints(grasps[0].grasp_pose.pose.position, gripper_offset_); //< offset the gripper position relative to the object.

    auto vector = quaternionToVector(grasps[0].grasp_pose.pose.orientation);

    ROS_INFO_STREAM("Pick vector: (" << vector.getX() << ", " << vector.getY() << ", " << vector.getZ() << ")");

    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = FRAME_ID;
    grasps[0].pre_grasp_approach.direction.vector = tf2::toMsg(vector);
    // grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;
    auto vectorA = grasps[0].pre_grasp_approach.direction.vector;
    ROS_INFO_STREAM("Pick vector A: (" << vectorA.x << ", " << vectorA.y << ", " << vectorA.z << ")");

    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = FRAME_ID;
    grasps[0].post_grasp_retreat.direction.vector = tf2::toMsg(vector * -0.5);
    // grasps[0].post_grasp_retreat.direction.vector.y = -0.5;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;
    auto vectorB = grasps[0].post_grasp_retreat.direction.vector;
    ROS_INFO_STREAM("Pick vector B: (" << vectorB.x << ", " << vectorB.y << ", " << vectorB.z << ")");

    openGripper(grasps[0].pre_grasp_posture);

    closeGripper(grasps[0].grasp_posture);

    auto s_qu = grasps[0].grasp_pose.pose.orientation;

    ROS_INFO_STREAM("Place q: (" << s_qu.w << ", " << s_qu.x << ", " << s_qu.y << ", " << s_qu.z << ")");

    // Call pick to pick up the object using the grasps given
    auto result = move_group_.pick(name, grasps);

    return moveItError(result);
}

bool Grasp::pickBar(const std::string &name, const geometry_msgs::Pose &object)
{
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // DEFINE POSE OF THE FETCH ROBOT ARM
    grasps[0].grasp_pose.header.frame_id = FRAME_ID;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, (M_PI / 2));
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position = object.position;
    addPoints(grasps[0].grasp_pose.pose.position, pick_bar_offset_); //< offset the gripper position relative to the object.
    addPoints(grasps[0].grasp_pose.pose.position, gripper_offset_);  //< offset the gripper position relative to the object.

    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = FRAME_ID;
    grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = FRAME_ID;
    grasps[0].post_grasp_retreat.direction.vector.y = -0.5;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);

    closeGripper(grasps[0].grasp_posture);

    // Call pick to pick up the object using the grasps given
    auto result = move_group_.pick(name, grasps);

    return moveItError(result);
}

bool Grasp::pickShelf(const std::string &name, const geometry_msgs::Pose &object)
{
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // DEFINE POSE OF THE FETCH ROBOT ARM
    grasps[0].grasp_pose.header.frame_id = FRAME_ID;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position = object.position;
    addPoints(grasps[0].grasp_pose.pose.position, pick_shelf_offset_); //< offset the gripper position relative to the object.
    addPoints(grasps[0].grasp_pose.pose.position, gripper_offset_);    //< offset the gripper position relative to the object.

    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = FRAME_ID;
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = FRAME_ID;
    grasps[0].post_grasp_retreat.direction.vector.x = -0.5;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);

    closeGripper(grasps[0].grasp_posture);

    // Call pick to pick up the object using the grasps given
    auto result = move_group_.pick(name, grasps);

    return moveItError(result);
}

bool Grasp::place(const std::string &name, const geometry_msgs::Pose &object)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    place_location[0].place_pose.header.frame_id = FRAME_ID;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    place_location[0].place_pose.pose = object;
    // addQuaternion(place_location[0].place_pose.pose.orientation, tf2::toMsg(orientation));
    addPoints(place_location[0].place_pose.pose.position, gripper_offset_); //< offset the gripper position relative to the object.

    auto vector = quaternionToVector(place_location[0].place_pose.pose.orientation);

    ROS_INFO_STREAM("Place vector: (" << vector.getX() << ", " << vector.getY() << ", " << vector.getZ() << ")");

    place_location[0].pre_place_approach.direction.header.frame_id = FRAME_ID;
    place_location[0].pre_place_approach.direction.vector = tf2::toMsg(vector);
    // place_location[0].pre_place_approach.direction.vector.y = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;
    auto vectorA = place_location[0].pre_place_approach.direction.vector;
    ROS_INFO_STREAM("Place vector A: (" << vectorA.x << ", " << vectorA.y << ", " << vectorA.z << ")");

    place_location[0].post_place_retreat.direction.header.frame_id = FRAME_ID;
    place_location[0].post_place_retreat.direction.vector = tf2::toMsg(vector * -0.5);
    // place_location[0].post_place_retreat.direction.vector.y = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;
    auto vectorB = place_location[0].post_place_retreat.direction.vector;
    ROS_INFO_STREAM("Place vector B: (" << vectorB.x << ", " << vectorB.y << ", " << vectorB.z << ")");

    openGripper(place_location[0].post_place_posture);

    auto s_qu = place_location[0].place_pose.pose.orientation;

    ROS_INFO_STREAM("Place q: (" << s_qu.w << ", " << s_qu.x << ", " << s_qu.y << ", " << s_qu.z << ")");

    // Set support surface as table2.
    //move_group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    auto result = move_group_.place(name, place_location);

    return moveItError(result);
}

bool Grasp::placeBar(const std::string &name, const geometry_msgs::Pose &object)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    place_location[0].place_pose.header.frame_id = FRAME_ID;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, (M_PI / 2.0));
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    place_location[0].place_pose.pose.position = object.position;
    addPoints(place_location[0].place_pose.pose.position, gripper_offset_); //< offset the gripper position relative to the object.

    place_location[0].pre_place_approach.direction.header.frame_id = FRAME_ID;
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    place_location[0].post_place_retreat.direction.header.frame_id = FRAME_ID;
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    openGripper(place_location[0].post_place_posture);

    auto result = move_group_.place(name, place_location);

    return moveItError(result);
}

bool Grasp::placeShelf(const std::string &name, const geometry_msgs::Pose &object)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    place_location[0].place_pose.header.frame_id = FRAME_ID;
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, -(M_PI / 2.0));
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    place_location[0].place_pose.pose.position = object.position;
    addPoints(place_location[0].place_pose.pose.position, gripper_offset_); //< offset the gripper position relative to the object.

    place_location[0].pre_place_approach.direction.header.frame_id = FRAME_ID;
    place_location[0].pre_place_approach.direction.vector.x = 1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    place_location[0].post_place_retreat.direction.header.frame_id = FRAME_ID;
    place_location[0].post_place_retreat.direction.vector.x = -0.5;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    openGripper(place_location[0].post_place_posture);

    auto result = move_group_.place(name, place_location);

    return moveItError(result);
}

void Grasp::addBottleObject(const std::string &name, const geometry_msgs::Pose &bottle)
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
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);
    collision_object.primitive_poses[0].orientation = tf2::toMsg(orientation);
    addPoints(collision_object.primitive_poses[0].position, bottle_offset_); //< offset the bottle position.
    collision_object.operation = collision_object.ADD;

    planning_scene_.applyCollisionObject(collision_object);
}

void Grasp::removeBottleObject(const std::string &name)
{
    std::vector<std::string> object_ids{name};
    planning_scene_.removeCollisionObjects(object_ids);
}

void Grasp::setupScene()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    //Left Bench
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "base_link";
        collision_object.id = "left_bench";

        double left_bench_z = 1.02;

        /* Define the primitive and its dimensions. */
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[0] = 0.9;
        collision_object.primitives[0].dimensions[1] = 1.6;
        collision_object.primitives[0].dimensions[2] = left_bench_z;

        /* Define the pose of the table. */
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position = fetch_offset_;
        collision_object.primitive_poses[0].position.x += 1.2;
        collision_object.primitive_poses[0].position.y += 3;
        collision_object.primitive_poses[0].position.z += left_bench_z / 2;
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }

    //Right Bench
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "base_link";
        collision_object.id = "right_bench";

        double right_bench_z = 1.02;

        /* Define the primitive and its dimensions. */
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[0] = 0.9;
        collision_object.primitives[0].dimensions[1] = 1.6;
        collision_object.primitives[0].dimensions[2] = right_bench_z;

        /* Define the pose of the table. */
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position = fetch_offset_;
        collision_object.primitive_poses[0].position.x += 1.2;
        collision_object.primitive_poses[0].position.y += 0;
        collision_object.primitive_poses[0].position.z += right_bench_z / 2;
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }

    //Bottom Shelf
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "base_link";
        collision_object.id = "bottom_shelf";

        double bottom_shelf_z = 0.1;

        /* Define the primitive and its dimensions. */
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[0] = 0.4;
        collision_object.primitives[0].dimensions[1] = 4.8;
        collision_object.primitives[0].dimensions[2] = bottom_shelf_z;

        /* Define the pose of the table. */
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position = fetch_offset_;
        collision_object.primitive_poses[0].position.x += 1.8;
        collision_object.primitive_poses[0].position.y += 1.5;
        collision_object.primitive_poses[0].position.z += 1.06 - (bottom_shelf_z / 2);
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }

    //Top Shelf
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "base_link";
        collision_object.id = "top_shelf";

        double top_shelf_z = 1;

        /* Define the primitive and its dimensions. */
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[0] = 0.4;
        collision_object.primitives[0].dimensions[1] = 4.8;
        collision_object.primitives[0].dimensions[2] = top_shelf_z;

        /* Define the pose of the table. */
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position = fetch_offset_;
        collision_object.primitive_poses[0].position.x += 1.8;
        collision_object.primitive_poses[0].position.y += 1.5;
        collision_object.primitive_poses[0].position.z += (1.06 + 0.3) + (top_shelf_z / 2);
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }

    //Back Shelf
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "base_link";
        collision_object.id = "back_shelf";

        double back_shelf_z = 2.36;

        /* Define the primitive and its dimensions. */
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[0] = 0.05;
        collision_object.primitives[0].dimensions[1] = 4.8;
        collision_object.primitives[0].dimensions[2] = back_shelf_z;

        /* Define the pose of the table. */
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position = fetch_offset_;
        collision_object.primitive_poses[0].position.x += 1.8 + (0.2 - 0.025);
        collision_object.primitive_poses[0].position.y += 1.5;
        collision_object.primitive_poses[0].position.z += back_shelf_z / 2;
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }

    //Left Shelf
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "base_link";
        collision_object.id = "left_shelf";

        double left_shelf_z = 0.3;

        /* Define the primitive and its dimensions. */
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[0] = 0.4;
        collision_object.primitives[0].dimensions[1] = 1.9;
        collision_object.primitives[0].dimensions[2] = left_shelf_z;

        /* Define the pose of the table. */
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position = fetch_offset_;
        collision_object.primitive_poses[0].position.x += 1.8;
        collision_object.primitive_poses[0].position.y += 2.9;
        collision_object.primitive_poses[0].position.z += 1.06 + (left_shelf_z / 2);
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }

    //Right Shelf
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "base_link";
        collision_object.id = "right_shelf";

        double right_shelf_z = 0.3;

        /* Define the primitive and its dimensions. */
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[0] = 0.4;
        collision_object.primitives[0].dimensions[1] = 1.9;
        collision_object.primitives[0].dimensions[2] = right_shelf_z;

        /* Define the pose of the table. */
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0].position = fetch_offset_;
        collision_object.primitive_poses[0].position.x += 1.8;
        collision_object.primitive_poses[0].position.y += 0.1;
        collision_object.primitive_poses[0].position.z += 1.06 + (right_shelf_z / 2);
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);
    }

    planning_scene_.applyCollisionObjects(collision_objects);
}

bool Grasp::moveBottle(geometry_msgs::Pose &current, geometry_msgs::Pose &target)
{
    ROS_INFO_STREAM("Adding bottle");
    addBottleObject("bottle", current);
    ros::WallDuration(0.1).sleep();

    ROS_INFO_STREAM("Picking bottle");
    if (!pick("bottle", current))
    {
        ROS_INFO_STREAM("Removing bottle\n");
        removeBottleObject("bottle");
        return false;
    }

    ros::WallDuration(0.1).sleep();
    ROS_INFO_STREAM("Placing bottle");

    if (!place("bottle", target))
    {
        ROS_INFO_STREAM("Removing bottle\n");
        removeBottleObject("bottle");
        return false;
    }

    ROS_INFO_STREAM("Removing bottle\n");
    removeBottleObject("bottle");

    return true;
}
bool Grasp::moveBottleToShelf(geometry_msgs::Pose &current, geometry_msgs::Pose &target)
{
    ROS_INFO_STREAM("Moving Bottle from Bar to Shelf");
    ROS_INFO_STREAM("Adding bottle");
    addBottleObject("bottle", current);
    ros::WallDuration(0.1).sleep();

    ROS_INFO_STREAM("Picking bottle");
    if (!pickBar("bottle", current))
    {
        ROS_INFO_STREAM("Removing bottle\n");
        removeBottleObject("bottle");
        return false;
    }

    ros::WallDuration(0.1).sleep();
    ROS_INFO_STREAM("Placing bottle");

    if (!placeShelf("bottle", target))
    {
        ROS_INFO_STREAM("Removing bottle\n");
        removeBottleObject("bottle");
        return false;
    }

    ROS_INFO_STREAM("Removing bottle\n");
    removeBottleObject("bottle");

    return true;
}
bool Grasp::moveBottleToBar(geometry_msgs::Pose &current, geometry_msgs::Pose &target)
{
    ROS_INFO_STREAM("Moving Bottle from Shelf to Bar");
    ROS_INFO_STREAM("Adding bottle");
    addBottleObject("bottle", current);
    ros::WallDuration(0.1).sleep();

    ROS_INFO_STREAM("Picking bottle");
    if (!pickShelf("bottle", current))
    {
        ROS_INFO_STREAM("Removing bottle\n");
        removeBottleObject("bottle");
        return false;
    }

    ros::WallDuration(0.1).sleep();
    ROS_INFO_STREAM("Placing bottle");

    if (!placeBar("bottle", target))
    {
        ROS_INFO_STREAM("Removing bottle\n");
        removeBottleObject("bottle");
        return false;
    }

    ROS_INFO_STREAM("Removing bottle\n");
    removeBottleObject("bottle");

    return true;
}

bool Grasp::graspingCallback(grasping::move::Request &req, grasping::move::Response &res)
{
    auto s_pose = req.current.position;
    auto e_pose = req.target.position;

    ROS_INFO_STREAM("Moving from pose: (" << s_pose.x << ", " << s_pose.y << ", " << s_pose.z << ")");
    ROS_INFO_STREAM("To pose: (" << e_pose.x << ", " << e_pose.y << ", " << e_pose.z << ")\n");
    res.success = moveBottle(req.current, req.target);

    ROS_INFO_STREAM("Grasping done\n");
    return true;
}
bool Grasp::shelfToBarCallback(grasping::move::Request &req, grasping::move::Response &res)
{
    auto s_pose = req.current.position;
    auto e_pose = req.target.position;

    ROS_INFO_STREAM("Moving from pose: (" << s_pose.x << ", " << s_pose.y << ", " << s_pose.z << ")");
    ROS_INFO_STREAM("To pose: (" << e_pose.x << ", " << e_pose.y << ", " << e_pose.z << ")\n");
    res.success = moveBottleToBar(req.current, req.target);

    ROS_INFO_STREAM("Grasping done\n");
    return true;
}
bool Grasp::barToShelfCallback(grasping::move::Request &req, grasping::move::Response &res)
{
    auto s_pose = req.current.position;
    auto e_pose = req.target.position;

    ROS_INFO_STREAM("Moving from pose: (" << s_pose.x << ", " << s_pose.y << ", " << s_pose.z << ")");
    ROS_INFO_STREAM("To pose: (" << e_pose.x << ", " << e_pose.y << ", " << e_pose.z << ")\n");
    res.success = moveBottleToShelf(req.current, req.target);

    ROS_INFO_STREAM("Grasping done\n");
    return true;
}

void Grasp::setGripperOffset(const geometry_msgs::Point &offset)
{
    gripper_offset_ = offset;
}

void Grasp::setPickOffset(const geometry_msgs::Point &shelf_offset, const geometry_msgs::Point &bar_offset)
{
    pick_bar_offset_ = bar_offset;
    pick_shelf_offset_ = shelf_offset;
}

void Grasp::setFetchOffset(const geometry_msgs::Point &offset)
{
    fetch_offset_ = offset;
}

void Grasp::setBottleOffset(const geometry_msgs::Point &offset)
{
    bottle_offset_ = offset;
}

void Grasp::addPoints(geometry_msgs::Point &pt1, const geometry_msgs::Point &pt2)
{
    pt1.x += pt2.x;
    pt1.y += pt2.y;
    pt1.z += pt2.z;
}

void Grasp::addQuaternion(geometry_msgs::Quaternion &qu1, const geometry_msgs::Quaternion &qu2)
{
    tf2::Quaternion qu1_tf;
    tf2::Quaternion qu2_tf;
    tf2::convert(qu1, qu1_tf);
    tf2::convert(qu2, qu2_tf);
    qu1_tf += qu2_tf;
    qu1 = tf2::toMsg(qu1_tf);
}

bool Grasp::moveItError(const moveit::planning_interface::MoveItErrorCode &ec)
{
    if (ec == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

tf2::Vector3 Grasp::quaternionToVector(const geometry_msgs::Quaternion &quaternion)
{
    tf2::Quaternion q_tf;
    tf2::convert(quaternion, q_tf);

    return q_tf.getAxis();
}