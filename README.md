Outline
=======

Reinforcement Learning game of pattern matching between a baxter and a human operator.

v: 0.1 (*very* alpha)

Bug @edran for questions.

Requirements
============

* `opencv`
* `ROS hydro`
* `baxter_interface` in development branch

Running the demo
================

* `rosrun learn_play calibrate.py`
* Start the vision (and calibrate it)
* Start the game

TODO
====

* ~~Write ALL THE THINGS IN PYTHON~~

* ~~Detect chessboard and pieces~~

* ~~Put left arm to position x, y (and calibrate (0, 0) wrt the board) at the start~~

* ~~Write calibration script for playing arm~~

* ~~Write picking movement~~ (ish)

* Write AI

Stuff
=====

* if number_pieces == odd, turn = baxter, else turn = human

* Left arm reaches point x, y, z+epsilon and then it goes to x, y, 0 (to release piece)
  Linear movements are essential.











