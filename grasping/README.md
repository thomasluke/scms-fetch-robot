# scms-fetch-robot - Grasping
(Author: Connor Bougton & Thomas Harrison)

General usage notes 
--------------------------------------------

RVIZ

- launch the RVIZ environment with 'roslaunch bar_simulation rviz.launch'
- run the ros grasping service with 'rosrun grasping grasping_service'
- run the ros integration (middle man program) with 'rosrun integration integration'
- send a bottle coordinate with 'rostopic pub /vision_poses geometry_msgs/PoseArray "header: [tab] [tab]'

GAZEBO

- launch the RVIZ environment with 'roslaunch bar_simulation gazebo.launch'
- run the ros grasping service with 'rosrun grasping grasping_service'
- run the ros integration (middle man program) with 'rosrun integration integration'
- send a bottle coordinate with 'rostopic pub /vision_poses geometry_msgs/PoseArray "header: [tab] [tab]'

---------------------------------

## About

The grasping package is used to control the movement of the fetch robot arm and gripper.

## Code

The grasping code uses MoveIt! and TF2 for the main control of the robot.
The code recieves the coordinates of the bottles used in the drink selected by the user. It will then move to each bottle and place them on the neighbouring bench for the bartender one at a time. It will then pick up and place the bottles back onto the shelf.

### ROS SERVICE

The custom ROS service contains a geometry pose which contains the current position and the target. Then it responds with a bool of success or failure. 

**There are three custom services**
1. grasping_service
2. grasping_service_shelf_to_bar
3. grasping_service_bar_to_shelf

-----------------------------------

## Dependencies

- MoveIT
- TF2
- Geometry_msgs
- roscpp
- message_generation
- /srv/move.srv
