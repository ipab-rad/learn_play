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
* `Python`

Running the demo
================

* `rosrun learn_play calibrate.py`
* `rosrun learn_play start_vision.py`
* `rosrun learn_play start_game.py`

TODO
====

* ~~Write ALL THE THINGS IN PYTHON~~

* ~~Detect chessboard and pieces~~

* ~~Put left arm to position x, y (and calibrate (0, 0) wrt the board) at the start~~

* ~~Write calibration script for playing arm~~

* ~~Write picking movement~~ (ish)

* Decide what kind of data to model

* Use pygame to draw grid

* Write AI (Right now it's e-greedy with e = 1 (that is, completely random)
