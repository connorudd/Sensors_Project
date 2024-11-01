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
%% Pick # 1 
end_effector_position = [0.2267, 0.0124, -0.0228];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% close 
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);
pause;
%% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Place #1 above
end_effector_position = [-0.0164, -0.2307, 0.0608];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Place #1 
end_effector_position = [-0.004, -0.2269, -0.022];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% OPEN
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);
pause;
%% Place #1 above
end_effector_position = [-0.0164, -0.2307, 0.0608];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Home
end_effector_position = [0.20705,0.008,0.1266];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Pick # 2 above 
end_effector_position = [0.2028, 0.0935, 0.0415];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Pick # 2  
end_effector_position = [0.20335,0.0976, -0.021];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% CLOSE
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);
pause;
%% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Place #2 above 
end_effector_position = [0.05572,-0.22389 , 0.04834];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Place #2
end_effector_position = [0.0633,-0.2194, -0.021];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% OPEN
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);
pause;
%% Place #2 above 
end_effector_position = [0.05572,-0.22389 , 0.04834];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Home
end_effector_position = [0.20705,0.008,0.1266];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Pick #3 above
end_effector_position = [0.263768, -0.03394, 0.0311498];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Pick #3 
end_effector_position = [0.263882, -0.03296, -0.0185];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% CLOSE
onOff = 1;
openClose = 1;
dobot.PublishToolState(onOff,openClose);
pause;
%% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Place #3 above 
end_effector_position = [0.1349, -0.2291, 0.024776];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Place #3  
end_effector_position = [0.1379, -0.2234, -0.018];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% OPEN
onOff = 1;
openClose = 0;
dobot.PublishToolState(onOff,openClose);
pause;
%% Place #3 above 
end_effector_position = [0.1349, -0.2291, 0.024776];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% traj 
end_effector_position = [0.1599, -0.181,0.1130];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Home
end_effector_position = [0.20705,0.008,0.1266];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
pause;
%% Turn off tool
onOff = 0;
openClose = 0;
dobot.PublishToolState(onOff,openClose);