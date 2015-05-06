# ardrone_control Repositiory

## Table of Contents

- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [Packages](#packages)
    - [ardrone_controller](#ardrone_controller)
    - [ardrone_sensor_fusion](#ardrone_sensor_fusion)
    - [ardrone_trajectory_generator](#ardrone_trajectory_generator)
    - [ardrone_simulator](#ardrone_simulator)
    - [ardrone_lib](#ardrone_lib)
- [Reading from Package](#readings)
- [Usage](#usage)
    - [Joystick Command](#joystick-command)
- [To do](#to-do)

## Introduction
This is a repository of a ROS package that controls the parrot AR.DRONE.
    - The ardrone_control package is a metapackage that holds different modules for controlling and simulating the Ar.Drone.
    - The ardrone_controller is a closed loop position feedback controller node.
    - The ardrone_sensor_fusion implements a sensor_fusion_algorithm.
    - The ardrone_trajectory_generator commands the desired position of the drone.
    - The ardrone_simulator is a bare-bones simulator that simulates the identified Ar.Drone system without any GUI.
    - The ardrone_lib is a package that has parameters and common modules used by other packages.
    - The ardrone_msgs defines the messages used in this metapackage.


## Dependencies
This package depends on:

* ROS Indigo Distribution (It should work with Hydro too, Catkin required)
* Ardrone Autonomy Package
* Joy Package
* Numpy, Scipy
* UTM Converter
* GeoMag Module

## Packages
### ardrone_controller
    The ardrone_control package implements a position feedback control for the Ar.Drone.
    Either PID, Trajectory_PID or Linear Controllers are Implemented.

### ardrone_sensor_fusion
    The ardrone_sensor_fusion package implements sensor fusion algorithm to
    estimate optimally the Ar.Drone state.
    An Extended Kalman filter is implemented for position estimation:
      - Odometry is used to estimate the position
      - GPS measurements are used to correct such estimations
    A Complementary filter is used to estimate orientation:
      - Gyroscope measurements are used to estimate orientation
      - Accelerometer and Magnetometer measurements are used to correct it.

### ardrone_trajectory_generator
    The ardrone_trajectory_generator package commands the reference set point of the ar drone.
    It allows the system to work in open loop (commanding velocity) or closed loop (commanding position).
    Way Points or a Trajectory function can be used to command the drone.

### ardrone_simulator
    The ardrone_simulator package simulates a Parrot Ar.Drone.
    It implements the identified transfer functions of the system,
    including an uncertainty model.
    The sensors are simulated considering bias drift, noise (Gaussian) and axis cross-coupling.
    The implemented sensors are:
      - Altimeter
      - Gyroscope
      - Magnetometer
      - Accelerometer
      - GPS
      - Velocity Estimation (From Ar.Drone on-board)
      - Orientation Estimation (From Ar.Drone on-board)

### ardrone_lib
    The ardrone_lib package contains common python modules used accross different packages.
    It also contains parameters used in different packages.

##Reading from Package
- The sensor_fusion node publishes its output in the topic /ardrone/estimation using a QuadrotorState msg defined in the ardrone_msgs package.
- The controller node publishes its output in the topic /cmd_vel using a Twist msg.
- The trajectory_generator publishes the reference in the topic /ardrone/reference using the same QuadrotorState msg.
- The simulator uses the same API as the ardrone_autonomy package for seamless integration. The simulated quadrotor publishes in the /ardrone/navdata, /ardrone/mag, /ardrone/fix, /ardrone/imu and /sonar_height topics. It also publishes the simulated truth in the /ardrone/ground_truth topic.

## Usage
### Joystick Command
When executed this module the joy_node will be running:

* Press 'X' for Drone to take-off
* Press 'Triangle' for Drone to land
* Move Right Analog towards Left (Right) to increase (decrease) Y-coordinate
* Move Right Analog towards Up (Down) to increase (decrease) X-coordinate
* Move Left Analog towards Right (Left) to yaw clockwise (anti-clockwise)
* Move Left Analog towards Up (Down) to increase (decrease) Z-coordinate goal
* Press start (select) arrow to turn controller on (off)
* Press L1 (L2) to turn to hover (flying) mode


## Modifications
To add your own module any module of the *.launch file can be commented

## To do

- Implement SO3 complementary (Magdwick, Mahoney) filters
- Implement Extended Kalman filter
- Add perturbations to the simulator
- Add uncertainty to the simulator
- Add a Chirp Service for Identification

