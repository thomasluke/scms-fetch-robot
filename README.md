# scms-fetch-robot - main

Fetch Bar Robot - Group 10  
--------------------------------------------

Launch
--------------------------------------------

RVIZ

- launch the RVIZ environment with 'roslaunch bar_simulation rviz.launch'
- run the ros grasping service with 'rosrun grasping grasping_service'
- run the ros integration (middle man program) with 'rosrun integration integration'
- run the ros vision with 'rosrun vision vision_service'
- run the drink menu with 'rosrun drink_menu drink_menu'

GRAZEBO

- launch the RVIZ environment with 'roslaunch bar_simulation gazebo.launch'
- run the ros grasping service with 'rosrun grasping grasping_service'
- run the ros integration (middle man program) with 'rosrun integration integration'
- run the ros vision with 'rosrun vision vision_service'
- run the drink menu with 'rosrun drink_menu drink_menu'

Packages Explained
-----------------------------------

![ROS Topics](https://github.com/thomasluke/scms-fetch-robot/blob/master/rosgraph.png)

**Simulation:**
This contains the Gazebo bar world file, ros launch files, and custom alcohol models.
> Bottles -> Connor (50%) / Natalie (50%)

> Launch Files -> Connor (100%)

> World File -> Connor (50%) / Natalie (50%)

**Drink Menu:**
The drink menu is the GUI interface for selecting a beverage.
It was made with QT GUI.
Upon selection of a beverage, the drink name and ingredients are published
> GUI -> Connor (100%)

> ROS -> Connor (100%)

**Vision:**
The vision code takes the drink ingredients and maps them to the AR codes.
It then uses the AR codes to determine the bottles in 3D space relative to the fetch robot.
The coordinates of the bottles are published.

**Integration:**
Integration is the middle man program between vision and grasping.
It is responsible for keeping track of the beverage ingredients, ensuring they are placed on the bar and then back on the shelf.
It takes the coordinate of an ingredient and the known bar position, then calls the ROS grasp service and awaits a response.
After all bottles are placed on the bar surface it then issues commands to place back on the shelf in the same order.

**Grasping**
Grasping is responsible for picking and placing the ingredients. 
It host custom ROS services for moving bottles from the shelf to the bar, and back.
> Launch Files -> Connor

> World File -> Connor / Natalie