# Assignment 1 – Equivalent Representations of Orientation Matrices

## Overview

This assignment for **Modelling and Control of Manipulators** explores the geometric foundations used to represent 3D orientations in robotics. It focuses on rotation matrices and their equivalent representations using the **angle-axis** and **Yaw-Pitch-Roll (YPR) Euler angles** conventions.

The MATLAB exercises implement conversions between these representations and examine their properties, including special and singular cases.

## Topics and Exercises

### 1. Angle-Axis to Rotation Matrix
- Represent a rotation using an axis of rotation and an angle.
- Implement the **Rodrigues formula** in `AngleAxisToRot.m`.
- Test the implementation with different axes and rotation angles.

### 2. Rotation Matrix to Angle-Axis
- Recover the axis and angle associated with a rotation matrix.
- Implement `RotToAngleAxis.m`.
- Validate the input rotation matrix through `IsRotationMatrix.m`.
- Consider special cases such as identity rotations and rotations with an angle of π.

### 3. Euler Angles (YPR) to Rotation Matrix
- Use the **Yaw-Pitch-Roll** convention, with the rotation sequence Z–Y–X.
- Implement `YPRToRot.m`.
- Test different combinations of yaw, pitch and roll, including configurations with pitch equal to π/2.

### 4. Rotation Matrix to Euler Angles
- Implement the inverse conversion in `RotToYPR.m`.
- Recover the YPR angles from a rotation matrix.
- Consider the singularity that occurs when `cos(pitch) = 0`.

### 5. Rotation Matrix to Angle-Axis Using Eigenvectors
- Use the eigenvector associated with eigenvalue 1 to identify the rotation axis.
- Compare the eigenvector-based result with the angle-axis representation obtained through the inverse equations.

### 6. Frame Tree
- Work with coordinate frames and their relationships.
- Represent transformations between frames using homogeneous transformation matrices.

## MATLAB Files

The MATLAB implementation is located in `mcm_assignment1_matlab/`.

| File | Purpose |
|---|---|
| `main.m` | Runs the test cases for the assignment exercises. |
| `include/AngleAxisToRot.m` | Converts an angle-axis representation into a rotation matrix. |
| `include/RotToAngleAxis.m` | Converts a rotation matrix into an angle-axis representation. |
| `include/YPRToRot.m` | Converts YPR Euler angles into a rotation matrix. |
| `include/RotToYPR.m` | Extracts YPR Euler angles from a rotation matrix. |
| `include/IsRotationMatrix.m` | Checks whether a matrix satisfies the rotation-matrix properties. |
| `include/RotToAngleAxis.m` | Supports the inverse angle-axis conversion used in the exercises. |

## How to Run

1. Open MATLAB.
2. Set the current folder to `mcm_assignment1_matlab/`.
3. Run `main.m`.

The script adds the `include` folder to the MATLAB path and evaluates the assignment test cases.

## Requirements and Notes

- MATLAB is required.
- The assignment instructions restrict the use of built-in MATLAB functions for the rotation-conversion implementations, apart from basic mathematical operations and the explicitly permitted functions.
- See `MCM_Assignment1-2.pdf` for the detailed questions, test cases and report discussion.
- `MCM_lab1_2025-26.pdf` contains the assignment template/instructions.

## Further Information

For the complete problem statements, mathematical derivations, test cases and discussion of the results, consult the PDF documents included in this folder.
