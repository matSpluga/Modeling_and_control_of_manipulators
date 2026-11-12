# Assignment 3 – Jacobian Matrices and Inverse Kinematics

## Overview

This assignment for **Modelling and Control of Manipulators** focuses on inverse-kinematics control of a 7-DoF industrial manipulator.

The task extends the geometric and kinematic models developed in the previous assignment by adding a tool frame and implementing a Cartesian control loop. The objective is to move the robot tool towards a specified goal pose by computing Cartesian errors, reference velocities and the corresponding joint velocities.

## Main Objectives

### 1. Tool Frame Definition
- Define a tool frame rigidly attached to the robot end-effector.
- Specify the tool orientation using Yaw-Pitch-Roll angles.
- Define the tool translation with respect to the end-effector.
- Implement `getToolTransformWrtBase()` in `geometricModel`.

### 2. Cartesian Error
- Define the goal frame with its desired position and orientation relative to the robot base.
- Compute the Cartesian error between the current tool pose and the goal pose.
- Represent the orientation error using the angle-axis representation.
- Express the angular and linear error components in the base frame.

### 3. Cartesian Reference Velocities
- Implement the Cartesian control law in `cartesianControl`.
- Use separate proportional gains for angular and linear errors.
- Compute the desired angular and linear reference velocities of the tool.

### 4. Inverse Kinematics
- Compute the tool Jacobian.
- Map the desired Cartesian velocity to joint velocities using the Jacobian pseudoinverse.
- Implement the inverse-kinematics control loop.

### 5. Kinematic Simulation
- Implement the joint-position integration function `KinematicSimulation()`.
- Update the robot configuration at each simulation step.
- Apply lower and upper joint limits.
- Plot the robot motion and monitor the tool and end-effector velocities.

## Repository Structure

The MATLAB project is located in `MCM_assignment3/`.

```text
MCM_assignment3/
├── src/
│   ├── main.m
│   └── include/
│       ├── BuildTree.m
│       ├── cartesianControl.m
│       ├── geometricModel.m
│       ├── kinematicModel.m
│       ├── KinematicSimulation.m
│       └── ...
├── MCM_lab3_25_26/
│   ├── main.tex
│   ├── Main/
│   ├── Resources/
│   └── ...
└── MCM_assignment3__report_.pdf
```

The folder also contains the report and LaTeX sources used to document the assignment.

## How to Run

1. Open MATLAB.
2. Set the current folder to `MCM_assignment3/src/`.
3. Run `main.m`.

The script adds the `include` directory to the MATLAB path, initializes the geometric and kinematic models, defines the tool and goal frames, and executes the inverse-kinematics simulation.

## Requirements

- MATLAB.
- The MATLAB source files and supporting classes included in `src/include/`.
- The provided `plotManipulators` utility for visualization.

## Further Information

For the full assignment description, parameter values, implementation discussion, plots and results, consult `MCM_assignment3__report_.pdf` and the assignment PDF files included in this folder.
