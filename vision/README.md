# scms-fetch-robot -vision

General usage notes 
--------------------------------------------
- launch the base code for bar_simulation  gazebo.launch the vision was though gazebo, this caused problems as if the lighting wasn't correct it cant read the ar tags correctly. 
- launch  the ar node with vision track.launch 
- rosrun vision vision 
- this should produce the pose of the ar tags that the robot can see/ and know which drinks it is looking at 
-----------------------------------
install ar_track_alavar 
-----------------------------------
It is important that the ar tags are made with rosrun ar_track_alvar createMarker and are named correctly in the in the dae files. 
ar tags where used as it is possible to know the pose of them once they are identified and then they are translated to the base link pose so they can be identified.
it does this by subscribing to the ar_track node which sends the pose and ar tag id information. 