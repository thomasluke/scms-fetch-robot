# scms-fetch-robot - Integration

General usage notes 
--------------------------------------------

- run the integration with 'rosrun integration integration'

--------------------------------------------

## About

The integration code takes the data published by the vision system and uses it to issue commands to the grasping code.

## Code

The drink menu uses QT GUI library for the GUI.
After a user clicks a menu item, it and its ingredients are published using a custom ROS msg.

### ROS MSG

The ROS msg contains a string for the drink name and an array of strings for the drink ingredients. 
