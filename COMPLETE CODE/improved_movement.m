%% 
% Main script for controlling the Dobot robot and processing images

% Initialize by moving the robot and processing an image
moveIT();
% ProcessImage();

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
    colours_detected = ProcessImage();
    green_collected = 1;
    blue_collected = 1;
    red_collected = 1;
    % Loop to detect colors and move objects accordingly
    while (colours_detected(1) >= 1 || colours_detected(2) >= 1 || colours_detected(3) >= 1)
        % Process image and detect colors
        colours_detected = ProcessImage();
        %% Green object handling
        if colours_detected(1) == 1
            
            disp('Green colour detected!');
            pause;
             pickABOVEpostionsGREEN = [
                [0.2454, -0.0273, 0.0191];
                [0.2782,0.0837, 0.01333];
                [0.2827, 0.00705,  0.00745];
                ];
            % Move above pick position
            dobot.PublishEndEffectorPose(pickABOVEpostionsGREEN(green_collected, :), [0, 0, 0]);
            pause;
            pickPositionsGREEN = [
                0.24488, -0.0270, -0.0228;
                 0.27924, 0.0839, -0.02754;
                0.28158, 0.00806, -0.03017;
            ];

            % Move to pick position
            dobot.PublishEndEffectorPose(pickPositionsGREEN(green_collected, :), [0, 0, 0]);
            pause;

            % Close the gripper to pick up the object
            dobot.PublishToolState(1, 1);  % Turn on and close the gripper
            pause;

            % Move above pick position
            dobot.PublishEndEffectorPose(pickABOVEpostionsGREEN(green_collected, :), [0, 0, 0]);
            pause;

            % Trajectory position (move back up)
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Move above place position
            placeAbovePosition = [-0.0436, -0.2465,  0.025];
            dobot.PublishEndEffectorPose(placeAbovePosition, [0, 0, 0]);
            pause;

            % Open the gripper to release the object
            dobot.PublishToolState(1, 0);  % Turn on and open the gripper
            pause;

            % Trajectory position (move back up)
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Return to home position
            dobot.PublishEndEffectorPose(homePosition, homeRotation);
            pause;
            
            green_collected = green_collected + 1;
        %% Blue object handling
        elseif colours_detected(2) == 1
            disp('Blue colour detected!');
            pause;
            pickABOVEpostionsBLUE = [
                [0.2028, 0.0935, 0.0415];
                [0.23609, 0.0720, 0.0135];
                [0.2808, -0.04461, 0.04366];

                ];

            % Move above pick position
            dobot.PublishEndEffectorPose(pickABOVEpostionsBLUE(blue_collected, :), [0, 0, 0]);
            pause;

             pickPostionsBLUE = [
                [0.20335, 0.0976, -0.021];
                [0.2355, 0.07176,  -0.021];
                [0.27505,-0.0438, -0.0199];
                ];

            % Move to pick position and grab the object
            dobot.PublishEndEffectorPose(pickPostionsBLUE(blue_collected, :), [0, 0, 0]);
            pause;

            % Close gripper 
            dobot.PublishToolState(1, 1);  % Close gripper
            pause;

            % Move above pick position
            dobot.PublishEndEffectorPose(pickABOVEpostionsBLUE(blue_collected, :), [0, 0, 0]);
            pause;

            % Trajectory to move upwards
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Place above #2
            placePosition2 = [0.103,-0.26 , 0.05];
            dobot.PublishEndEffectorPose(placePosition2, [0, 0, 0]);
            pause;

            % Release object by opening gripper
            dobot.PublishToolState(1, 0);  % Open gripper
            pause;

            % Trajectory to move upwards
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Return to home position
            dobot.PublishEndEffectorPose(homePosition, homeRotation);

            blue_collected = blue_collected + 1;
            pause;

        %% Red object handling
        elseif colours_detected(3) == 1
            disp('Red colour detected!');
            pause;
            pickABOVEPositionsRED = [
                
                [0.2423, 0.010, 0.0383];
                [0.2892, 0.03642, 0.0501];
                [ 0.32091,  -0.0464, 0.0148];

            ];
            % Move above pick position
            dobot.PublishEndEffectorPose(pickABOVEPositionsRED(red_collected, :), [0, 0, 0]);
            pause;

            pickPositionsRED = [
                
                [0.2384, 0.0102, -0.0233];
                [0.29417, 0.033164, -0.0180];
                [0.32050, -0.0487, -0.02353];

            ];

            % Move to pick position and grab the object
            dobot.PublishEndEffectorPose(pickPositionsRED(red_collected, :), [0, 0, 0]);
            pause;

            % close 
            dobot.PublishToolState(1, 1);  % Close gripper
            pause;

            % Move above pick position
            dobot.PublishEndEffectorPose(pickABOVEPositionsRED(red_collected, :), [0, 0, 0]);
            pause;

            % Trajectory to move upwards
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Place above #3
            placePosition3 = [0.24418,  -0.1722, 0.0230];
            dobot.PublishEndEffectorPose(placePosition3, [0, 0, 0]);
            pause;

            % Release object by opening gripper
            dobot.PublishToolState(1, 0);  % Open gripper
            pause;

            % Trajectory to move upwards
            dobot.PublishEndEffectorPose(trajPosition, [0, 0, 0]);
            pause;

            % Return to home position
            dobot.PublishEndEffectorPose(homePosition, homeRotation);

            red_collected = red_collected + 1;
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
    
    % colour_test = [
    % 1,1,1;
    % 1,1,1;
    % 0,1,1;
    % 0,0,1;
    % 0,0,0;
    % ];
    % 
    % colours_detected = colour_test(test_count);


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