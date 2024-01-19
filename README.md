# Overview

This repository contains code and documentation for controlling the Xarm6 robot. The project covers various aspects, including forward kinematics, inverse kinematics, numerical inverse kinematics, Jacobians, dynamics, and trajectory generation.

## Introduction
This code serves as a practical implementation of robotic kinematics and dynamics, focusing on the application of forward kinematics, inverse kinematics, workspace analysis, and Jacobian matrices. Specifically designed for the xarm6 machete robot, the code aims to provide a comprehensive tool for understanding and applying theoretical concepts from a robotics class to real-world scenarios.

## System Requirements
Python environment with necessary libraries NumPy, Matplotlib, Pybullet. 

## Code Overview:
### 1. Forward Kinematics
#### 1.1 Frame Transformations
The forward kinematics module is structured as a class, encapsulating various functions for frame transformations. Functions _0T1 to _5T6 calculate transformation matrices from one frame to another using Denavit-Hartenberg parameters. These matrices enable the determination of the robot's end effector position and orientation.

#### 1.2 Position and Orientation
Functions _jointpos, _ori, and _endeff provide insights into the robot's configuration. _jointpos outputs the positions of each joint with respect to frame 0. _ori calculates the orientation of the end effector using Euler angles, and _endeff yields the transformation matrix for the end effector (Frame-6).

#### 1.3 Jacobian Matrix
The _Jacobian function computes the Jacobian matrix, a crucial tool for understanding the robot's kinematics and aiding in tasks such as inverse kinematics and workspace analysis.

### 2. Inverse Kinematics
The Newton-Raphson algorithm is implemented for solving inverse kinematics. The iterative approach refines joint angles to achieve a desired end effector position.

### 3. Visualization
The code leverages the Pybullet library for visualization, allowing users to observe the robot's motion and configuration in a simulated environment. The visualization component enhances the understanding of the robot's behavior and aids in debugging.


### 4. Inverse Kinematics with Jacobian
An additional feature incorporates the Jacobian matrix into the inverse kinematics solution. This enhancement improves the convergence speed and accuracy of joint angle adjustments.

### 5. Inverse Dynamics
The code includes the capability for inverse dynamics calculations, allowing for the determination of joint torques given external forces applied to the end effector.

### 6. Trajectory Generation
A trajectory generation module employs cubic polynomials to plan smooth paths for the robotic arm. The implementation facilitates the robot's movement along predefined trajectories.


## Visualization
clone the following repo:
```python
git clone https://github.com/bulletphysics/bullet3/tree/master
```


## References:
5.1 MODERN ROBOTICS MECHANICS, PLANNING, AND CONTROL.
5.2 Numerical Methods for Inverse Kinematics by Niels Joubert, UC Berkeley, CS184

