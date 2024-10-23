% % step 1: run roscore 
% rosinit
% nodes = rostopic("list");
% disp(nodes);
% 
% % % Do this once we know where to publish it  
% % % Initialize ROS
% % rosinit;  % Connect to ROS master
% % 
% % Create a publisher for the Dobot command topic
% % pub = rospublisher('/dobot/command', 'std_msgs/Float64MultiArray');
% % 
% % Create a message for the publisher
% % msg = rosmessage(pub);
% % 
% % Define the target position (joint angles or Cartesian coordinates)
% % Here we define a simple example with joint angles [joint1, joint2, joint3, joint4]
% % targetPosition = [0.5, 0.5, 0.5, 0];  % Example values, adjust as necessary
% % 
% % Populate the message with the target position
% % msg.Data = targetPosition;
% % 
% % Publish the command to move the robot
% % send(pub, msg);
% % 
% % Display a message indicating the robot is moving
% % disp('Moving the Dobot to the target position...');
% % 
% % Optionally, wait for a confirmation or some time to allow the movement
% % pause(5);  % Adjust the pause duration based on the movement time
% % 
% % Shutdown ROS connection
% % rosshutdown;
% 
% Step 1: Initialize ROS connection to the Dobot's ROS master
% ipaddress = '172.28.114.129';  % Replace with your ROS master IP
% rosinit(ipaddress, 11311);      % Port 11311 is the default ROS master port
% 
% Step 2: Create ROS Action Client for the Dobot's arm controller
% [dobotArm, dobotGoalMsg] = rosactionclient('dobot_arm_controller/joint_trajectory_action');
% waitForServer(dobotArm);  % Wait until the action client connects to the action server
% 
% Step 3: Set the joint names for the Dobot robot
% dobotGoalMsg.Trajectory.JointNames = {'joint1', 'joint2', 'joint3', 'joint4'};  % Replace with your Dobot's actual joint names
% 
% Step 4: Create Joint Trajectory Points
% Point 1: Initial position (neutral joint angles)
% tjPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% tjPoint1.Positions = [0 0 0 0];  % Initial neutral position
% tjPoint1.Velocities = zeros(1,4);  % Zero velocity
% tjPoint1.TimeFromStart = rosduration(1.0);  % 1 second from the start
% 
% Point 2: Move to a target position
% tjPoint2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% tjPoint2.Positions = [0.5 -0.2 0.3 0.1];  
% tjPoint2.Velocities = zeros(1,4);  % Zero velocity (constant velocity)
% tjPoint2.TimeFromStart = rosduration(2.0);  % 2 seconds from the start
% 
% Step 5: Assign trajectory points to the goal message
% dobotGoalMsg.Trajectory.Points = [tjPoint1, tjPoint2];
% 
% Step 6: Send the goal to the Dobot's action server and wait for completion
% sendGoalAndWait(dobotArm, dobotGoalMsg);
% 
% Step 7: Shutdown the ROS connection after the movement is completed
% rosshutdown;


% Initialize Serial Port
dobot = serialport("COM3", 115200);
% Set line terminator
configureTerminator(dobot, "CR/LF");


writeline(dobot, "VER");
pause(1); % Wait for response
if dobot.NumBytesAvailable > 0
    response = readline(dobot);
    disp("Response from Dobot: " + response);
else
    disp("No response received from Dobot.");
end


% % Enable motors to allow movement
% try
%     writeline(dobot, "M17");
%     pause(1);  % Allow motors to initialize
% 
%     % Check if there are bytes available to read
%     if dobot.NumBytesAvailable > 0
%         response = readline(dobot);
%         disp("Response from Dobot: " + response);  % Display the response
% 
%         % Check for specific response
%         if contains(response, "OK")
%             disp("Motors enabled successfully.");
%         else
%             disp("Failed to enable motors: " + response);
%         end
%     else
%         disp("No response received from Dobot after M17 command.");
%     end
% catch ME
%     disp("Failed to enable motors:");
%     disp(ME.message);
% end
% 
% pickupPos = [239.9, 46.3, -12.24];
% placePos = [167.57, 189, -13.2];
% pickupHeight = 10;
% placeHeight = 10;
% 
% % Move the robot to a specific (X, Y, Z) position
% function moveTo(dobot, x, y, z)
%     command = sprintf("G1 X%.2f Y%.2f Z%.2f F2000", x, y, z); % G1 for linear move
%     disp("Sending command: " + command); % Debug output
%     try
%         writeline(dobot, command);
%         pause(2); % Allow time for the robot to complete the move
% 
%         % Check response after move
%         if dobot.NumBytesAvailable > 0
%             response = readline(dobot);
%             disp("Response after move: " + response);
%         end
%     catch ME
%         disp("Error during movement:");
%         disp(ME.message);
%     end
% end
% 
% function controlGripper(dobot, state)
%     if state == 1
%         writeline(dobot, "M4 T1"); % Close gripper
%         disp("Gripper closed");
%     else
%         writeline(dobot, "M4 T0"); % Open gripper
%         disp("Gripper opened");
%     end
%     pause(1); % Allow time for the gripper to actuate
% end
% 
% % Movement sequence
% moveTo(dobot, 220, -20, 135); % Move to a test position
% pause;
% moveTo(dobot, pickupPos(1), pickupPos(2), pickupPos(3) + pickupHeight); % Move above pickup position
% moveTo(dobot, pickupPos(1), pickupPos(2), pickupPos(3)); % Move down to pickup position
% controlGripper(dobot, 1); % Close gripper
% moveTo(dobot, pickupPos(1), pickupPos(2), pickupPos(3) + pickupHeight); % Move back up after grabbing
% moveTo(dobot, placePos(1), placePos(2), placePos(3) + placeHeight); % Move to place position (above)
% moveTo(dobot, placePos(1), placePos(2), placePos(3)); % Move down to place position
% controlGripper(dobot, 0); % Open gripper
% moveTo(dobot, placePos(1), placePos(2), placePos(3) + placeHeight); % Move back up after placing
% 
% % Optional: Return to home position
% try
%     writeline(dobot, "G28"); % Go to home position
%     pause(2); % Allow time for the movement to complete
%     disp("Returned to home position");
% catch ME
%     disp("Error while returning to home position:");
%     disp(ME.message);
% end
% 
% % Close the serial port
% try
%     clear dobot;   % Clear the object from memory
%     disp("Serial port closed successfully");
% catch ME
%     disp("Error while closing serial port:");
%     disp(ME.message);
% end
