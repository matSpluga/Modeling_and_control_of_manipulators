# Assignment 2 – Manipulator Geometry and Direct Kinematics

## Overview

This assignment for **Modelling and Control of Manipulators** focuses on the geometric modelling and direct kinematics of an industrial **7-DoF serial manipulator**.

The MATLAB implementation builds the robot model from the transformations between consecutive links, updates the geometry according to the joint configuration, computes transformations between reference frames and evaluates the manipulator Jacobian.

## Main Objectives

### 1. Geometric Model
- Define the fixed transformation matrices between consecutive joint frames through `BuildTree()`.
- Specify the joint types (revolute or prismatic).
- Define the reference frames consistently with the joint axes and the robot CAD model.

### 2. Direct Geometry
- Implement `updateDirectGeometry()` in `geometricModel`.
- Update the link-to-link homogeneous transformation matrices as a function of the joint positions.
- Compute the transformation from the manipulator base to a selected frame using `getTransformWrtBase()`.
- Compute transformations between arbitrary frames using `getTransformWrtRandomFrame()`.

### 3. Manipulator Configuration and Simulation
- Evaluate the manipulator at the specified initial and final joint configurations.
- Interpolate the joint variables and visualize the resulting robot motion.
- Use the provided `plotManipulators` class for plotting.

### 4. Jacobian Computation
- Implement `getJacobianOfLinkWrtBase()` in `kinematicModel`.
- Compute the Jacobian of an individual link with respect to the base frame.
- Implement `updateJacobian()` to obtain the end-effector Jacobian.
- Distinguish the contributions of revolute and prismatic joints.

### 5. End-Effector Velocity
- Use the Jacobian to map joint velocities to the end-effector twist.
- Express angular and linear velocity components in the requested reference frame.

## Repository Structure

The main MATLAB project is located in `MCM_lab2_25_26/`.

```text
MCM_lab2_25_26/
├── src/
│   ├── main.m
│   └── include/
│       ├── BuildTree.m
│       ├── geometricModel.m
│       ├── kinematicModel.m
│       └── plotManipulators.m
├── MCM_lab2_25_26/
│   ├── main.tex
│   ├── Main/
│   ├── Resources/
│   └── ...
├── Report_Lab2.pdf
└── MCM_lab2_25_26.pdf
```

The report and LaTeX source files are also included in the project folder.

## How to Run

1. Open MATLAB.
2. Set the current folder to `MCM_lab2_25_26/src/`.
3. Run `main.m`.

The script adds the `include` directory to the MATLAB path, builds the geometric model, runs the configuration and Jacobian calculations, and invokes the provided plotting class.

## Requirements

- MATLAB.
- The project MATLAB source files and the provided plotting utilities.
- The CAD model and assignment documentation supplied in this folder.

## Further Information

For the complete exercise statements, mathematical explanations, computed transformations, Jacobian results and discussion, consult `Report_Lab2.pdf` and the assignment PDF files included in this folder.
