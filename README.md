## Software and Hardware Requirements
The software requirements for running the simulation code inculded in this 
Repository are Peter Corke's Robotics Toolbox, and specifically the DoBot 
Magician UTS robot model. This model includes all of the .ply files of each
robotic link and the method to create the robot in the correct orientation 
and correct DH parameters. Along with this the MATLAB support package for 
USB cameras for connection and access to the Intel RealSense RGB-D camera. 
This coding solution also requires wireless connection to a Raspberry Pi 4 
to access the DoBot Magician's ROS topics.

## Coding Instructions
# DoBot Robot Control and Object Detection

This project involves controlling a DoBot Magician robot to detect and 
manipulate coloured objects (red, green, blue) using an Intel RealSense 
RGB-D camera for image processing. The robot uses ROS (Robot Operating 
System) to communicate commands and state, while image processing is done 
in MATLAB to identify and locate objects by colour.

## Software and Hardware Requirements

### Required Software:
- **MATLAB** with:
  - Peter Corke's Robotics Toolbox for MATLAB, specifically the UTS model, 
    which includes 3D models and correct DH parameters for a DoBot Magician.
  - MATLAB Support Package for USB Webcams for connecting to the Intel 
    RealSense RGB-D camera.
- **ROS (Robot Operating System)**, with:
  - Connection capability to communicate with the DoBot Magician via a 
    Raspberry Pi 4.
- **Intel RealSense SDK** (for the camera).

### Required Hardware:
- **DoBot Magician** robotic arm
- **Intel RealSense RGB-D Camera** (preferably the D435 model)
- **Raspberry Pi 4** configured as a ROS node connected to the DoBot 
    Magician

## Setup Instructions

### Camera Setup
1. Follow the [Intel RealSense SDK setup instructions]
    (https://github.com/IntelRealSense/librealsense) to install necessary 
    drivers and libraries for the RGB-D camera.
2. Connect the Intel RealSense camera to your computer via USB, ensuring 
    that it is recognized and accessible.

### DoBot Magician and Raspberry Pi Setup
1. Ensure that the Raspberry Pi is connected to the DoBot Magician and 
    configured with ROS.
2. Connect the Raspberry Pi to your network and confirm it is accessible 
    from your device.

### MATLAB Configuration
1. Install the required MATLAB toolboxes:
   - Robotics Toolbox by Peter Corke for UTS, including the DoBot Magician 
    model.
   - MATLAB Support Package for USB Webcams.
2. Connect your device to the Raspberry Pi’s IP address for ROS 
   communication (`rosinit('192.168.27.1');` in the code).

## Running the Simulation

1. Run `simulation_file.m` in the Simulation folder to initialize and 
   assess the simulation without hardware.

## Running the Demo

To run the live demo with the physical DoBot robot and camera:

1. Open `improved_movement.m` in the COMPLETE CODE folder.
2. Ensure all devices (DoBot, camera, Raspberry Pi, and your laptop) are 
    properly connected and turned on.
3. Run the script by selecting **Run Section** in MATLAB. The script 
    initializes the robot, captures images, and processes object colours 
    for manipulation.

### Important Notes for Execution
- Wait for each robot action to complete before pressing Enter to proceed. 
    Each movement can take a variable time to finish and pressing Enter 
    prematurely may cause errors.
- The demo follows a sequential process:
  - **Initialization**: The robot moves to a home position.
  - **Colour Detection**: An image is captured, and colours are detected to 
    identify red, green, and blue objects.
  - **Object Manipulation**: Based on detected colours, the robot moves to 
    each object’s location, picks it up, and places it at a target position.

## Project Files Overview

- **`improved_movement.m`**: The main control file for the robot’s 
    movements, including object detection and manipulation.
- **`ProcessImage.m`**: Image processing code to capture an RGB image, 
    detect colours using HSV thresholds, and label detected objects.
- **`DoBotMagician.m`**: Contains methods for interacting with the DoBot 
    robot via ROS, such as publishing end-effector poses and controlling 
    the gripper.
- **`The movement.m file contains the code for Demo-Day, whilst 
  improved_movement is the edited code we finalised the following week. 














