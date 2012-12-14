# ardrone_interface
# =================
Interface based on pygame and ROS to control the AR.Drone

## Dependencies:
https://github.com/AutonomyLab/ardrone_autonomy

## Install Guide:
- Install ardrone_autonomy
- Clone this repository
- rosmake this module
- Run it by: roslaunch ardrone_interface interface.launch

## Controls:

* `w`		elevator up	
* `s`	:elevator down
* `a`	:yaw left
* `d`	:yaw right
* `up`	:pitch down --> forward
* `down`	:pitch up   --> backward
* `left`	:Roll left
* `right`	:Roll right
* `space`	 :take off / land
* `c`	:toggle between front camera and bottom camera
* `r`	:reset/flat trim --> Works in both flight modes 
* `t`   :reset the tracker, only when tracking. This is actually not necessary. Only for debugging.
* `-`	:decrease sensibility
* `+`	:increase sensibility
* `m`   :toggle between manual_flightmode and autonomous_flightmode
* `b`   :show batterystatus in percent
* `enter` :confirms the box you've selected in the videofeed.

## Tracking an object:

We've implemented an other project (github.com/Ronan0912/ros_opentld). We used the trackernode named tld_tracker
for its algorithm, which is OpenTLD. You can select a bounding box with the mouse in the videofeed of the ARdrone.
If right it presents a nice green box, when ready you can confirm it with the 'enter' key.
It sends the bounding box to the tld_tracker node. For doing this you have to run the tld_tracker node as well of course.

## Autonomous_flightmode:

When changed to autonomous flying the interface wil try to steer the ARdrone in
a manner that the object, the one you are tracking, will stay in the gray rectangle.

### !! Warning !! 
This is work in progress and at the moment not all dimensions and steering
commands are implemented.

