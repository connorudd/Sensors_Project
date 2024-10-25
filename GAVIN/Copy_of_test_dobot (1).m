clear all;
clc;
close all;
rosshutdown;
%% Start Dobot Magician Node
rosinit('192.168.27.1');

%% Start Dobot ROS
dobot = DobotMagician();

%% Publish custom end effector pose
end_effector_position = [207.05,10.08,126.6];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% Turn on tool
% Open tool
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);
%%
% Close tool
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);

%% Turn off tool
onOff = 0;
openClose = 0;
dobot.PublishToolState(onOff,openClose);

