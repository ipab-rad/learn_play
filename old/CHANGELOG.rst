1.0.0 (2014-5-1)
---------------------------------
- Adds joint_position_waypoints example program
- Updates ik_service_client to validate unpacked results and seed types
- Updates gripper_cuff_control to automatically calibrate on gripper type change
- Updates navigator_io to use navigator wheel_changed signal
- Updates all examples to verify robot software version by default when enabling
- Updates all examples using gripper to verify gripper firmware version
- Updates joint_recorder to record at 100Hz
- Updates joint_velocity_wobbler to run at 500Hz

0.7.0 (2013-11-21)
---------------------------------
- Creation of baxter_examples repository from sdk-examples/examples.
- Adds joint torque springs examples.
- Adds gripper cuff control example.
- Adds gripper action client example.
- Package restructure in support of Catkin expected standards.
- Adds launch files for examples using action servers or the joystick.
- Adds gripper position playback to joint trajectory file playback example.
- Removes camera_control example, now located in baxter_tools repository.
- Removes getch usage as means of exiting example programs (latency).
- Fixes joint position file playback looping. Loops now start at correct playback start.
- Fixes head movement during exit of wobbler example.
- Adds timeouts to action client's wait_for calls making sure the action servers are running.
- Fixes D-Pad mapping for ps3 joysticks.
- Adds success verification for result of joint trajectory file playback.
