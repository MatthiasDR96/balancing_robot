# Balancing robot

## Source code

* '1_system_models': Contains the dynamic (non-linear and linear) 
models of the balancing robot. 

* '2_sensors': Contains some scripts for reading the IMU and 
performing some basic sensor fusion techniques to obtain a stable angle 
measurement

* '3_actuators': Contains some scripts for actuating the DC 
motors via the L298N motor driver

* '4_controllers': Contains some controllers to stabilize the robot.

## Scripts

* 'dynamics_field' visulaizes the differential field of the dynamics 
and the used controller

* 'stability_analysis' calculates the eigenvalues and thus stability 
of the dynamics with and without controllers

* 'simulate' simulates the dynamics with controller and plots the state

* 'optimize_control_params' optimizes control values as an 
auto-tune would do