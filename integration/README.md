# scms-fetch-robot - Integration
(Author: Connor Bougton)

General usage notes 
--------------------------------------------

- run the integration with 'rosrun integration integration'

--------------------------------------------

## About

The integration code takes the data published by the vision system and uses it to issue commands to the grasping code.
It is the middle man program between vision and grasping, and is responsible for keeping track of the bottls, ensuring they are placed on the bar and then back on the shelf.

## Code

After recieving bottle coordinates, the code calls a custom ROS services in the grasping package (***grasping_service_shelf_to_bar***).
If this is completed successfully the bottle is known to be on the shelf.
The code then adds an offset and places bottle on the bar, until all the required bottles are placed.

After all bottles are placed on the bar surface the code then attempts to place the bottles back on the shelf.
It picks the most recently placed bottle then calls another custom ROS services in the grasping package (***grasping_service_bar_to_shelf***).
It continues removing the bottles until they are all back on the shelf.

It then waits for a new set of coordinates from vision.

## Dependencies

- TF2
- Geometry_msgs
- roscpp
- message_generation
- grasping/srv/move.srv