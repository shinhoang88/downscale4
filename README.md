# Control Algorithms for the 7-DOF Heavy-duty Redundant Manipulators
## 1) PD control synchronizing with Gazebo including Gravity and Friction Compensation:

![Optional Text](https://github.com/shinhoang88/media_files/blob/master/All7JointsPDcontrol1.gif)

## 2) Active stiffness control (Cartesian stiffness control):
Depend on the task's requirement (the manipulator need to be stiff or soft), we can modulate the End Effector Cartesian stiffness:

https://github.com/shinhoang88/downscale4/blob/master/src/pmaccartesianstiffness.cpp

In this video, the EE stiffness is modulated as it can move freely along Y-axis while is still stiff along X and Z axes.

![Optional Text](https://github.com/shinhoang88/media_files/blob/master/FreelyAlongYaxis_StifferAlongXZAxis.gif)

