ardrone_interface
=================
Interface based on pygame and ROS to control the AR.Drone

Dependencies:
https://github.com/AutonomyLab/ardrone_autonomy

Install Guide:
- Install ardrone_autonomy
- Clone this repository
- rosmake this module
- Run it by: rosrun ardrone_interface interface.py

Controls:

- 'w'		elevator up	
- 's'		elevator down
- 'a'		yaw left
- 'd'		yaw right

- 'UP'		pitch down --> forward
- 'DOWN'  	pitch up   --> backward
- 'LEFT'	Roll left
- 'RIGHT'	Roll right

- 'space' 	take off / land
- 'c'		toggle between front camera and bottom camera
- 'r'		reset/flat trim
- '-'		decrease sensibility
- '+'		increase sensibility
