# scms-fetch-robot - Grasping
(Author: Connor Bougton & Thomas Harrison)

General usage notes 
--------------------------------------------

RVIZ

- launch the RVIZ environment with 'roslaunch bar_simulation rviz.launch'
- run the ros grasping service with 'rosrun grasping grasping_service'
- run the ros integration (middle man program) with 'rosrun integration integration'
- send a bottle coordinate with 'rostopic pub /vision_poses geometry_msgs/PoseArray "header: [tab] [tab]'

GRAZEBO

- launch the RVIZ environment with 'roslaunch bar_simulation gazebo.launch'
- run the ros grasping service with 'rosrun grasping grasping_service'
- run the ros integration (middle man program) with 'rosrun integration integration'
- send a bottle coordinate with 'rostopic pub /vision_poses geometry_msgs/PoseArray "header: [tab] [tab]'

-----------------------------------

## Dependencies

- MoveIT
- TF2
- roscpp
- message_generation
- /srv/move.srv