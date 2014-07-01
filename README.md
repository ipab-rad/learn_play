Outline
=======

* Write ALL THE THINGS IN PYTHON 

* Start baxter

* Put right arm to position x, y

* Detect chessboard (save coordinates)


ROS
===

Node from left camera publishes images. (so message is image)

Node that subscribes to camera and analyzes pattern. This publishes
coordinates of pieces. (so message is some structure of floats)

Node that controls `left_arm` (`cmd_vel`?) will move arm, pick stuff
etc. It will subscribe to analyzer.






