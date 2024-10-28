% connectRealSense.m
% Script to connect MATLAB to RealSense camera via ROS

clear all;
clc;
close all;

rosshutdown;  % Shutdown any active ROS connection

% % Initialize ROS (connect to ROS master)
% rosinit;
% 
% % Define subscribers for color and depth topics
% depthSub = rossubscriber('/camera/depth/image_rect_raw', 'sensor_msgs/Image');
% colorSub = rossubscriber('/camera/color/image_raw', 'sensor_msgs/Image');
% 
% % Pause briefly to ensure connection is stable
% pause(1);
% 
% % Receive messages
% depthMsg = receive(depthSub, 10);  % 10-second timeout
% colorMsg = receive(colorSub, 10);
% 
% % Convert ROS messages to MATLAB images
% depthImage = readImage(depthMsg);
% colorImage = readImage(colorMsg);
% 
% % Display the images
% figure;
% subplot(1, 2, 1);
% imshow(colorImage);
% title('Color Image');
% 
% subplot(1, 2, 2);
% imshow(depthImage, []);
% title('Depth Image');
% 
% Shutdown ROS when done
% rosshutdown;


% connectRealSenseAndDetectGreen_Debug.m
disp('Starting ROS initialization...');
rosinit('172.31.160.238');

% Define the color image subscriber
disp('Setting up subscriber for color image...');
colorSub = rossubscriber('/camera/color/image_raw', 'sensor_msgs/Image');
pause(2);  % Allow time for the subscriber to connect

% Check if the topic is connected
disp('Checking if the subscriber is connected to the topic...');
if ~isvalid(colorSub)
    error('The color image subscriber is not valid. Check the topic name and connection.');
else
    disp('Subscriber successfully connected to the topic.');
end

try
    % Attempt to receive a message from the color image topic
    disp('Attempting to receive a color image message...');
    colorMsg = receive(colorSub, 20);  % Increase timeout if needed
    disp('Color image message received successfully.');

    % Convert ROS message to MATLAB image
    colorImage = readImage(colorMsg);
    disp('Converted ROS message to MATLAB image.');

    % Convert the RGB image to HSV
    hsvImage = rgb2hsv(colorImage);
    disp('Converted RGB image to HSV.');

    % Define HSV thresholds for green
    greenHueMin = 0.25;  % Adjust for the hue range of green
    greenHueMax = 0.45;
    greenSatMin = 0.4;   % Minimum saturation for green
    greenValMin = 0.2;   % Minimum brightness for green

    % Create a binary mask for green regions
    disp('Creating binary mask for green regions...');
    greenMask = (hsvImage(:,:,1) >= greenHueMin) & (hsvImage(:,:,1) <= greenHueMax) & ...
                (hsvImage(:,:,2) >= greenSatMin) & (hsvImage(:,:,3) >= greenValMin);

    % Apply the mask to the original image
    greenDetectedImage = colorImage;
    greenDetectedImage(repmat(~greenMask, [1, 1, 3])) = 0;
    disp('Applied mask to isolate green areas.');

    % Display original and green-detected images
    figure;
    subplot(1, 2, 1);
    imshow(colorImage);
    title('Original Color Image');
    subplot(1, 2, 2);
    imshow(greenDetectedImage);
    title('Green-Detected Image');
    disp('Displayed images successfully.');

catch ME
    % Display error message and additional information
    fprintf('Error occurred: %s\n', ME.message);
    fprintf('Make sure the camera node is active, and topics are being published correctly.\n');
end

% Shutdown ROS
disp('Shutting down ROS...');
rosshutdown;


% % connectRealSense.m
% disp('Initializing RealSense pipeline...');
% pipe = realsense.pipeline();
% 
% try
%     % Start streaming with default settings
%     disp('Starting streaming...');
%     profile = pipe.start();
% 
%     % Retrieve and display device name
%     dev = profile.get_device();
%     name = dev.get_info(realsense.camera_info.name);
%     disp(['Camera name: ', name]);
% 
%     % Capture a few frames to test the pipeline
%     for i = 1:5
%         disp(['Capturing frame ', num2str(i), '...']);
%         fs = pipe.wait_for_frames();
%         if fs.is_valid()
%             disp('Frame is valid.');
%         else
%             disp('Frame is not valid.');
%         end
%     end
% 
%     % Stop streaming
%     disp('Stopping streaming...');
%     pipe.stop();
%     disp('RealSense pipeline test completed successfully.');
% 
% catch ME
%     % Display any errors encountered during the test
%     disp('Error encountered during RealSense pipeline test:');
%     disp(ME.message);
% end
