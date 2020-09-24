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
