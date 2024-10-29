moveIT();
%% move the robot 
function moveIT()
clear all;
clc;
close all;
rosshutdown;
%% Start Dobot Magician Node
 rosinit('192.168.27.1');
% 
 %% Start Dobot ROS
 dobot = DobotMagician();

%% Home
end_effector_position = [0.20705,0.008,0.1266];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% OPEN
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);
pause;

colours_detected = ProcessImage();

%% Red colour option
if colours_detected(1) == 1
disp('Green colour detected!');
% Pick # 1 
end_effector_position = [0.2267, 0.0124, -0.0228];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% close 
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);
pause;
% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Place #1 above
end_effector_position = [-0.0164, -0.2307, 0.0608];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Place #1 
end_effector_position = [-0.004, -0.2269, -0.022];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% OPEN
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);
pause;
% Place #1 above
end_effector_position = [-0.0164, -0.2307, 0.0608];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Home
end_effector_position = [0.20705,0.008,0.1266];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;

%% Blue colour option
elseif colours_detected(2) == 1 && colours_detected(1) == 0
disp('Blue colour detected!');
% Pick # 2 above 
end_effector_position = [0.2028, 0.0935, 0.0415];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Pick # 2  
end_effector_position = [0.20335,0.0976, -0.021];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% CLOSE
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);
pause;
% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Place #2 above 
end_effector_position = [0.05572,-0.22389 , 0.04834];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Place #2
end_effector_position = [0.0633,-0.2194, -0.021];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% OPEN
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);
pause;
% Place #2 above 
end_effector_position = [0.05572,-0.22389 , 0.04834];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Home
end_effector_position = [0.20705,0.008,0.1266];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;

%% Red colour option
elseif colours_detected(1) == 0 && colours_detected(2) == 0 && colours_detected(3) == 1
disp('Red colour detected!');
% Pick #3 above
end_effector_position = [0.263768, -0.03394, 0.0311498];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Pick #3 
end_effector_position = [0.263882, -0.03296, -0.0185];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% CLOSE
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);
pause;
% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Place #3 above 
end_effector_position = [0.1349, -0.2291, 0.024776];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Place #3  
end_effector_position = [0.1379, -0.2234, -0.018];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% OPEN
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);
pause;
% Place #3 above 
end_effector_position = [0.1349, -0.2291, 0.024776];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
% Home
end_effector_position = [0.20705,0.008,0.1266];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
else 
% Turn off tool
onOff = 0;
openClose = 0;
dobot.PublishToolState(onOff,openClose);
end 

disp('Demonstration Completed (move it move it');
end

%% Fucntion for Image processing
function colours_detected = ProcessImage()

    clf;
    clc;
    
    % Capture an RGB image from the camera
    camList = webcamlist;  % List available webcams
    disp('Available cameras:');
    disp(camList);

    % Connect to the D435 camera (replace index if necessary)
    cam = webcam(2);  % Adjust the index if needed

    % Capture the image
    rgbImage = snapshot(cam);

    % rgbImage = imadjust(rgbImage, stretchlim(rgbImage), []);  % Stretch limits

    % Display the captured image
    figure;
    imshow(rgbImage);
    title('Captured RGB Image');

    % Convert the RGB image to HSV for better color segmentation
    hsvImage = rgb2hsv(rgbImage);

    % Step 2: Define Adjusted HSV Thresholds for Red, Green, and Blue
    % Red Mask (Hue around 0 or 1, high saturation, and brightness)
    redMask = ((hsvImage(:,:,1) >= 0.95 | hsvImage(:,:,1) <= 0.05) & ...
           hsvImage(:,:,2) >= 0.3 & hsvImage(:,:,3) >= 0.3);

    % Green Mask (Hue around 0.33, moderate to high saturation and brightness)
    greenMask = (hsvImage(:,:,1) >= 0.2 & hsvImage(:,:,1) <= 0.48 & ...
             hsvImage(:,:,2) >= 0.2 & hsvImage(:,:,3) >= 0.2);

    % Blue Mask (Hue around 0.67, high saturation and brightness)
    blueMask = (hsvImage(:,:,1) >= 0.5 & hsvImage(:,:,1) <= 0.7 & ...
            hsvImage(:,:,2) >= 0.2 & hsvImage(:,:,3) >= 0.2);

    % Perform Morphological Operations to Clean Up Masks
    redMask = imopen(redMask, strel('disk', 5));
    greenMask = imopen(greenMask, strel('disk', 5));
    blueMask = imopen(blueMask, strel('disk', 5));

    % Label Connected Components and Find Bounding Boxes
    redStats = regionprops(redMask, 'BoundingBox', 'Centroid');
    greenStats = regionprops(greenMask, 'BoundingBox', 'Centroid');
    blueStats = regionprops(blueMask, 'BoundingBox', 'Centroid');

    % Check if Each Color is Detected
    isRedDetected = ~isempty(redStats);
    isGreenDetected = ~isempty(greenStats);
    isBlueDetected = ~isempty(blueStats);

    colours_detected = [isGreenDetected, isBlueDetected, isRedDetected];

    % Display the Detection Results in the Command Window
    fprintf('Red Detected: %d\n', isRedDetected);
    fprintf('Green Detected: %d\n', isGreenDetected);
    fprintf('Blue Detected: %d\n', isBlueDetected);

    % Step 6: Display the Results
    figure;
    imshow(rgbImage);
    hold on;

    % Draw Bounding Boxes for Red Objects
    for i = 1:length(redStats)
        rectangle('Position', redStats(i).BoundingBox, 'EdgeColor', 'r', 'LineWidth', 2);
        plot(redStats(i).Centroid(1), redStats(i).Centroid(2), 'r*', 'MarkerSize', 10);
    end

    % Draw Bounding Boxes for Green Objects
    for i = 1:length(greenStats)
        rectangle('Position', greenStats(i).BoundingBox, 'EdgeColor', 'g', 'LineWidth', 2);
        plot(greenStats(i).Centroid(1), greenStats(i).Centroid(2), 'g*', 'MarkerSize', 10);
    end

    % Draw Bounding Boxes for Blue Objects
    for i = 1:length(blueStats)
        rectangle('Position', blueStats(i).BoundingBox, 'EdgeColor', 'b', 'LineWidth', 2);
        plot(blueStats(i).Centroid(1), blueStats(i).Centroid(2), 'b*', 'MarkerSize', 10);
    end

    title('Detected Red, Green, and Blue Objects');
    hold off;

    % Clean up
    clear cam;

end
