Objectives in this project were:
1 . Move the robot forward and stop at some distance from the wall
2. Follow the encountered wall
Our approach was to build small behaviors like 'move forward', 'stop', etc. and organize them into a composite behavior that fulfilled the objectives. To make our robot follow a wall, we developed a 'follow wall' behavior based on PID control.
We worked with a robot called e-puck, a differential-drive non-holonomic robot, to develop and deploy simple concepts in autonomous behavior based robotics, namely:
1. Detecting if open space is available in front of the robot and stopping the robot if it is not
2. Making the robot follow a wall (more generally, an obstacle)
Webots, a software for simulating mobile robots, served as our development platform, where we designed and re_ned controllers that were deployed on the real robot. Along with a simulation of a dimensionally and mechanically accurate model of the e-puck, Webots also provided us several libraries that allowed us to work on an abstract level of robot variables like wheel speeds and distance sensor values without having to bother about the underlying electronics and construction of the robot. We were therefore able to develop and think in terms of discrete behavioral modules to fulfill our tasks.
