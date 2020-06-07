# scms-fetch-robot - Drink Menu
(Author: Connor Bougton)

General usage notes 
--------------------------------------------

- run the drink menu with 'rosrun drink_menu drink_menu'

--------------------------------------------

## About

The drink menu package is the GUI for the whole project.
It is where the end user interacts with the robot.


## Code

The drink menu uses QT GUI library for the GUI.
After a user clicks a menu item, it and its ingredients are published using a custom ROS msg.

### ROS MSG

The ROS msg contains a string for the drink name and an array of strings for the drink ingredients. 

## Dependencies

- QT GUI
- roscpp
- message_generation
- /msg/drink.msg