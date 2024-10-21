% step 1: run roscore 
rosinit
nodes = rostopic("list");
disp(nodes);

%% Do this once we know where to publish it  
% % Initialize ROS
% rosinit;  % Connect to ROS master

% Create a publisher for the Dobot command topic
pub = rospublisher('/dobot/command', 'std_msgs/Float64MultiArray');

% Create a message for the publisher
msg = rosmessage(pub);

% Define the target position (joint angles or Cartesian coordinates)
% Here we define a simple example with joint angles [joint1, joint2, joint3, joint4]
targetPosition = [0.5, 0.5, 0.5, 0];  % Example values, adjust as necessary

% Populate the message with the target position
msg.Data = targetPosition;

% Publish the command to move the robot
send(pub, msg);

% Display a message indicating the robot is moving
disp('Moving the Dobot to the target position...');

% Optionally, wait for a confirmation or some time to allow the movement
pause(5);  % Adjust the pause duration based on the movement time

% Shutdown ROS connection
rosshutdown;
