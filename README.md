# Control Algorithms for the 7-DOF Heavy-duty Redundant Manipulators
## 1) PD control synchronizing with Gazebo including Gravity and Friction Compensation:
Friction compensate model: LuGre model based

![Optional Text](https://github.com/shinhoang88/media_files/blob/master/All7JointsPDcontrol1.gif)

## 2) Active stiffness control (Cartesian stiffness control):
Depend on the task's requirement (the manipulator need to be stiff or soft), we can modulate the End Effector Cartesian stiffness:

https://github.com/shinhoang88/downscale4/blob/master/src/pmaccartesianstiffness.cpp

In these videos, the EE stiffness is modulated so that it can move freely along Y-axis while it is still stiff along X and Z axes.

### a) Simulation result:

![Optional Text](https://github.com/shinhoang88/media_files/blob/master/stiffness_largexz_smally.gif)

### b) Experiment:

![Optional Text](https://github.com/shinhoang88/media_files/blob/master/FreelyAlongYaxis_StifferAlongXZAxis.gif)

## 3) Master-Slave teleoperation:

Master device: Phantom Omni (6-DOF)

Slave: 7-DOF heavy-duty manipulator

For doing a remote task, the manipulator can be controlled by the master device. In the task-space of the manipulator, it can avoid the obstacle using its redundancy by changing the arm angle.

### a) Simulation result:

![Optional Text](https://github.com/shinhoang88/media_files/blob/master/teleoperation_sim.gif)

### b) Experiment:

Teleoperation with position and orientation:

![Optional Text](https://github.com/shinhoang88/media_files/blob/master/Teleoperation_test1.gif)

Changing the arm angle: 

![Optional Text](https://github.com/shinhoang88/media_files/blob/master/Teleoperation_test2.gif)

## 4) A New Torque Minimization Method for Heavy-duty Redundant Manipulators Used in Nuclear Decommissioning Tasks:

This propose a method to optimize the control torque of heavy-duty redundant manipulators used for dismantling nuclear power plants:
Publication link (Intelligent Service Robotics Journal, June 2021): 
https://link.springer.com/article/10.1007/s11370-021-00369-4?fbclid=IwAR3jAI8fR9KF3hXJEAc2UAzcSdmKRs8COPxPR_UjFViFLxrrwdPMLkHMzn8

### a) Simulation:

For the given straight line trajectory:

https://github.com/shinhoang88/downscale4/blob/master/src/codestraightline.cpp

For the given quarter-circle trajectory:

https://github.com/shinhoang88/downscale4/blob/master/src/codequartercircle.cpp

### b) Experiments:

![Optional Text](https://github.com/shinhoang88/media_files/blob/master/TorqueMinimization_v2_1.gif)

![Optional Text](https://github.com/shinhoang88/media_files/blob/master/TorqueMinimization_v2_2.gif)

- Author        : Phi Tien Hoang
- E-mail        : phitien@skku.edu
- Organization  : Robotory-SKKU-S.Korea
