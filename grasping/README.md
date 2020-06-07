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

The grasping package is used to control the movement of the fetch robot arm and girpper.

## Code

The grasping code uses MoveIt! and TF2 for the main control of the robot.
The code recieves the coordinates of the bottles used in the drink selected by the user. It will then move to each bottle and place them on the neighbouring bench for the bartender one at a time. It will then pick up and place the bottles back onto the shelf.

### ROS MSG

The ROS service contains a geometry pose which contains the current position and the target. Then it responds with a bool of success or failure. 

-----------------------------------

## Dependencies

- MoveIT
- TF2
- roscpp
- message_generation
- /srv/move.srv