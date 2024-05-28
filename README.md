# SwarmRobotics2023-24ECE448
Code repository for ECE448/449 Senior Design Project at Miami University 

Group Members: Rob Lytton, Mika Burmester, Ryan Campbell, Ben Eicher

The goal of our project is to implement a boids like algorithm using physical robots

![swarm-robotics-poster pptx](https://github.com/RobLytton/SwarmRobotics2023-24ECE448/assets/92745408/648cc010-160b-4aef-a178-8852679fd9b7)

## Running the Code

The code for this project was developed and tested in a Linux environment (Ubuntu and RaspberryPI OS).

#### boid-simulation

The boid-simulation folder contains a simulation of the robots written in Javascript. To run this, simply open the index.html file using a web browser. The simulation was never completed, so there is not much to see here.

#### camera-calibration

The camera-calibration folder contains a program for determining the distortion coefficients of a camera using OpenCV and a checkboard calibration pattern.

#### motor-control

The motor-control folder contains code for controlling motors with encoders using a RaspberryPI. To compile the code, run the make command in the directory. Pigpio must be installed for the program to compile.

Running motor_control controls the motors and sets the speeds based on input from a FIFO file pipe. The motor_control program must be run with sudo to properly work with the GPIO interface. The wiring diagram below shows how the motors and encoders are wired for our project.

![wiring](https://github.com/RobLytton/SwarmRobotics2023-24ECE448/assets/9685690/34fc46a2-45ee-4f47-925a-72de688a57a0)

Running send_motor_command sends a motor command using the FIFO pipe that motor_control reads from. This program takes two integer arguments (-255 to 255) to set the left and right motor speeds. This program must be run with sudo to properly interact with motor_control.

The motor_command.h header file contains code for interacting with the FIFO pipe used by motor_control. This header can be included in other programs to enable interaction between them and motor_control. An example of this usage can be found in send_motor_command.c.

#### tag-detection

The tag-detection directory contains code utilizing OpenCV to detect the custom tags (OctoTags) built for our project. The makefile for this requires OpenCV to be installed and configured using pkg-config (see below for more information).

The robot_detector program detects robots in a frame using the tag detector. The tag_demo program demonstrates the capabilities of the tag detector and provides control of various parameters for the detector.

The detector has various parameters that control the detection of tags. The colors' hue, range, saturation, and value will likely need to be adjusted based on the lighting of the environment. They are listed below:

Color - Selects the color range to configure.

Hue - Sets the center hue for the color range.

Range - Sets the range of hue values for the color range.

Saturation - Sets the saturation threshold for the color range.

Value - Sets the value theshold for the color range.

Area Threshold - Sets the minimum area required for a contour to be considered as a feature. Decreasing this value increases detections.

Ellipse Threshold - Sets the minimum 'ellipseness' needed for a feature to be considered an ellipse. Decreasing this value increases detections.

Triangle Threshold - Sets the minimum 'triangulariy' needed for a feature to be considered a triangle. Decreasing this value increases detections.

Corner Distance Error Limit - Sets the maximum distance a corner can be from the center of a tag in percentage.

Corner Area Scale Limit - Sets the maximum size error a corner can have from the expected area based on the center ellipse.

Corner Angle Error Limit - Sets the maximum angle a corner can be from the expected value.

## OpenCV Setup

OpenCV needs to be installed and configured properly to compile the tag-detection code. Instructions for this can be found in OpenCV-Setup.pdf located in the top directory of the repository. The instructions reference opencv4.pc which is contained in the same directory as the instructions.
