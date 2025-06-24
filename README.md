# Integrated Kinematic and Dynamic Modeling of  SCARA Robot Using MATLAB, Python, and ROS 2

This project also includes a complete symbolic and numerical analysis of the robot’s **kinematics**, **Jacobian**, and **dynamics** using **MATLAB**, complemented by motion profile generation with **Python**. The analytical approach provides deeper insight into robot behavior under realistic dynamic constraints.

## MATLAB Contributions

- **Kinematic Modeling (DH-Method):**
  - Symbolic generation of homogeneous transformation matrices.
  - Computation of forward and inverse kinematics using algebraic and geometric methods.
  - Workspace analysis based on over 200,000 evaluated configurations.
  - Thermally induced length changes and required axis corrections.

- **Jacobian and Force Analysis:**
  - Full derivation of the Jacobian matrix with respect to linear and angular velocity.
  - Singular value decomposition (SVD) for singularity detection.
  - Force and torque computation at each joint under given TCP loads.

- **Lagrangian Dynamics:**
  - Symbolic derivation of Lagrange equations using kinetic and potential energy.
  - Computation of:
    - Mass matrix \( M(q) \)
    - Coriolis and centrifugal matrix \( C(q, \dot{q}) \)
    - Gravity vector \( G(q) \)
  - Numerical evaluation of \(\boldsymbol{\tau}(t)\) over time based on real motion profiles.

- **MATLAB Scripts Include (only main files):**
  - `Auslegung_jacobi_iteractive_caipo_english.m`
  - `Auslegung_lagrange_caipo_english.m`
  - `Auslegung_Lagrange_iteractive_english.m`
  - `Auslegung_RoboterAuslegung_english.m`

## Python Contribution

- **Motion Profile Generation:**
  - Implemented custom trajectory generation using **jerk-limited motion profiles**.
  - Created time-continuous profiles for position, velocity, acceleration, and jerk.
  - Exported profiles for use in dynamic torque evaluation in MATLAB.

## ROS 2 Integration

- Complete URDF/XACRO structure of the SCARA robot.
- MoveIt 2 configuration for planning and collision avoidance.
- RViz visualization of robot structure and movements.




## Author

Manuel Caipo  
Msc.Advanced Precision Engineering (APE)  
Hochschule Furtwangen  
Email: manuelcaipocc@outlook.com