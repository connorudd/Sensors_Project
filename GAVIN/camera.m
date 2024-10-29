clear all;
clc;
close all;
rosshutdown;  % Shutdown any active ROS connection
rosinit('192.168.27.1');
cameraTopic = '/camera/color/image_raw';  % ROS topic for RGB images
depthTopic = '/camera/depth/image_raw';  % ROS topic for depth images
% Set up subscribers for RGB and depth data
rgbSub = rossubscriber(cameraTopic, 'sensor_msgs/Image');
depthSub = rossubscriber(depthTopic, 'sensor_msgs/Image');
% Retrieve RGB and depth images
rgbMsg = receive(rgbSub);
depthMsg = receive(depthSub);
% Convert ROS messages to MATLAB images
rgbImage = readImage(rgbMsg);
depthImage = readImage(depthMsg);
% Display the images side-by-side
figure;
subplot(1, 2, 1); imshow(rgbImage); title('RGB Image');
subplot(1, 2, 2); imshow(mat2gray(depthImage)); title('Depth Image');