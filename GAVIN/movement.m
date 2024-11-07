%% 
% Main script for controlling the Dobot robot and processing images

% Initialize by moving the robot and processing an image
% moveIT();
 ProcessImage();

%% Main function to control robot movements
function moveIT()
    % Clear previous settings and initialize ROS connection
    clear all;
    clc;
    close all;
    rosshutdown;  % Shutdown any active ROS nodes
    rosinit('192.168.27.1');  % Initialize ROS and connect to robot

    % Create an instance of DobotMagician for control commands
    dobot = DobotMagician();

    % Move robot to the home position
    homePosition = [0.20705, 0.008, 0.1266];
    trajPosition = [0.1599, -0.181, 0.1130];
    homeRotation = [0, 0, 0];
    dobot.PublishEndEffectorPose(homePosition, homeRotation);
    pause;

    % Open the gripper
    dobot.PublishToolState(1, 0);  % Turn on and open the gripper
    pause;

    % Loop to detect colors and move objects accordingly
    for i = 1:3
        % Process image and detect colors
        colours_detected = ProcessImage();

        %% Green object handling
        if colours_detected(1) == 1
            disp('Green colour detected!');

            % Move to pick position
            pickPosition = [0.2267, 0.0124, -0.0228];
            dobot.PublishEndEffectorPose(pickPosition, [0, 0, 0]);
            pause;

            % Close the gripper to pick up the object
            dobot.PublishToolState(1, 1);  % Turn on and close the gripper
            pause;

            % Trajectory position (move back up)
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Move above place position
            placeAbovePosition = [-0.0164, -0.2307, 0.0608];
            dobot.PublishEndEffectorPose(placeAbovePosition, [0, 0, 0]);
            pause;

            % Place the object at the desired location
            placePosition = [-0.004, -0.2269, -0.022];
            dobot.PublishEndEffectorPose(placePosition, [0, 0, 0]);
            pause;

            % Open the gripper to release the object
            dobot.PublishToolState(1, 0);  % Turn on and open the gripper
            pause;

            % Move back to the initial position above the place location
            dobot.PublishEndEffectorPose(placeAbovePosition, [0, 0, 0]);
            pause;

            % Trajectory position (move back up)
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Return to home position
            dobot.PublishEndEffectorPose(homePosition, homeRotation);
            pause;

        %% Blue object handling
        elseif colours_detected(2) == 1
            disp('Blue colour detected!');

            % Move above pick position
            pickAbovePosition = [0.2028, 0.0935, 0.0415];
            dobot.PublishEndEffectorPose(pickAbovePosition, [0, 0, 0]);
            pause;

            % Move to pick position and grab the object
            pickPosition = [0.20335, 0.0976, -0.021];
            dobot.PublishEndEffectorPose(pickPosition, [0, 0, 0]);
            pause;

            % Close gripper 
            dobot.PublishToolState(1, 1);  % Close gripper
            pause;

            % Trajectory to move upwards
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Place above #2
            placePosition2 = [0.05572,-0.22389 , 0.04834];
            dobot.PublishEndEffectorPose(placePosition2, [0, 0, 0]);
            pause;

            % Place object at designated position
            placePosition = [0.0633, -0.2194, -0.021];
            dobot.PublishEndEffectorPose(placePosition, [0, 0, 0]);
            pause;

            % Release object by opening gripper
            dobot.PublishToolState(1, 0);  % Open gripper
            pause;

            % Place above #2
            dobot.PublishEndEffectorPose(placePosition2, [0, 0, 0]);
            pause;

            % Trajectory to move upwards
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Return to home position
            dobot.PublishEndEffectorPose(homePosition, homeRotation);
            pause;

        %% Red object handling
        elseif colours_detected(3) == 1
            disp('Red colour detected!');

            % Move above pick position
            pickAbovePosition = [0.263768, -0.03394, 0.0311498];
            dobot.PublishEndEffectorPose(pickAbovePosition, [0, 0, 0]);
            pause;

            % Move to pick position and grab the object
            pickPosition = [0.263882, -0.03296, -0.013];
            dobot.PublishEndEffectorPose(pickPosition, [0, 0, 0]);
            pause;

            % close 
            dobot.PublishToolState(1, 1);  % Close gripper
            pause;

            % Trajectory to move upwards
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Place above #3
            placePosition3 = [0.1417, -0.2215, 0.03493];
            dobot.PublishEndEffectorPose(placePosition3, [0, 0, 0]);
            pause;

            % Place object at designated position
            placePosition = [0.1379, -0.2234, -0.018];
            dobot.PublishEndEffectorPose(placePosition, [0, 0, 0]);
            pause;

            % Release object by opening gripper
            dobot.PublishToolState(1, 0);  % Open gripper
            pause;

            % Place above #3
            dobot.PublishEndEffectorPose(placePosition3, [0, 0, 0]);
            pause;

            % Trajectory to move upwards
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Return to home position
            dobot.PublishEndEffectorPose(homePosition, homeRotation);
            pause;
        end
    end

    % Turn off the gripper
    dobot.PublishToolState(0, 0);  % Turn off tool
    disp('Demonstration Completed (move it move it)');
end

%% Function to process image and detect colors
%% Fucntion for Image processing
function colours_detected = ProcessImage()

    clf;
    clc;

    % Capture an RGB image from the camera
    % camList = webcamlist;  % List available webcams
    % disp('Available cameras:');
    % disp(camList);

    % Connect to the D435 camera (replace index if necessary)
    cam = webcam(3);  % Adjust the index if needed

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