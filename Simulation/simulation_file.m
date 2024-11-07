%% Workspace Setup Function
clf;
clc;

% Create and plot robot at specified joint position
% Setup robot workspace
robot = DobotMagician();
q0 = [0, pi/6, pi/4, pi/2, 0];
workspace = [-0.4, 0.4, -0.4, 0.4, 0, 0.4];
scale = 0.5;
robot.model.plot(q0,'workspace',workspace,'scale',scale);
axis(workspace);
hold on;

%% Plot and detect color for green square
% Extract data from the green square ply file, scale it to the environment and transform
% it to the desired position for the pickup instance. 
[f, v, data] = plyread('green_square.ply', 'tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
square_start_position = [0.25, 0, 0];
square_scale_factor = 0.002;
square_v_scaled = v * square_scale_factor;
square_v_transformation = square_v_scaled + square_start_position;
square1 = trisurf(f, square_v_transformation(:, 1), square_v_transformation(:, 2), square_v_transformation(:, 3), ...
    'FaceVertexCData', vertexColours, ...
    'FaceColor', 'interp', ...
    'EdgeColor', 'none');

% Extract data from the green square ply file, scale it to the environment and transform
% it to the desired position for the end-effector instance. 
[f, v, data] = plyread('green_square.ply', 'tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
square_start_position2 = [0.25, 0, 0];
square_scale_factor2 = 0.002;
square_v_scaled2 = v * square_scale_factor2;
square_v_transformation2 = square_v_scaled2 + square_start_position;
square2 = trisurf(f, square_v_transformation2(:, 1), square_v_transformation2(:, 2), square_v_transformation2(:, 3), ...
    'FaceVertexCData', vertexColours, ...
    'FaceColor', 'interp', ...
    'EdgeColor', 'none');
hideObj(square2);

% Extract data from the green square ply file, scale it to the environment and transform
% it to the desired position for the place instance. 
[f, v, data] = plyread('green_square.ply', 'tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
square_start_position3 = [0, 0.25, 0];
square_scale_factor3 = 0.002;
square_v_scaled3 = v * square_scale_factor3;
square_v_transformation3 = square_v_scaled3 + square_start_position3;
square3 = trisurf(f, square_v_transformation3(:, 1), square_v_transformation3(:, 2), square_v_transformation3(:, 3), ...
    'FaceVertexCData', vertexColours, ...
    'FaceColor', 'interp', ...
    'EdgeColor', 'none');
hideObj(square3);

%% Plot and detect color for blue octagon
% Extract data from the blue octagon ply file, scale it to the environment and transform
% it to the desired position for the pickup instance. 
[f, v, data] = plyread('blue_octagon.ply', 'tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
octagon_start_position = [0.25, 0.07, 0];
octagon_scale_factor = 0.002;
octagon_v_scaled = v * octagon_scale_factor;
octagon_v_transformation = octagon_v_scaled + octagon_start_position;
octagon1 = trisurf(f, octagon_v_transformation(:, 1), octagon_v_transformation(:, 2), octagon_v_transformation(:, 3), ...
    'FaceVertexCData', vertexColours, ...
    'FaceColor', 'interp', ...
    'EdgeColor', 'none');

% Extract data from the blue octagon ply file, scale it to the environment and transform
% it to the desired position for the end-effector instance. 
[f, v, data] = plyread('blue_octagon.ply', 'tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
octagon_start_position2 = [0.1, 0.25, 0];
octagon_scale_factor2 = 0.002;
octagon_v_scaled2 = v * octagon_scale_factor2;
octagon_v_transformation2 = octagon_v_scaled2 + octagon_start_position2;
octagon2 = trisurf(f, octagon_v_transformation2(:, 1), octagon_v_transformation2(:, 2), octagon_v_transformation2(:, 3), ...
    'FaceVertexCData', vertexColours, ...
    'FaceColor', 'interp', ...
    'EdgeColor', 'none');
hideObj(octagon2);

% Extract data from the blue octagon ply file, scale it to the environment and transform
% it to the desired position for the place instance. 
[f, v, data] = plyread('blue_octagon.ply', 'tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
octagon_start_position3 = [0.1, 0.25, 0];
octagon_scale_factor3 = 0.002;
octagon_v_scaled3 = v * octagon_scale_factor3;
octagon_v_transformation3 = octagon_v_scaled3 + octagon_start_position3;
octagon3 = trisurf(f, octagon_v_transformation3(:, 1), octagon_v_transformation3(:, 2), octagon_v_transformation3(:, 3), ...
    'FaceVertexCData', vertexColours, ...
    'FaceColor', 'interp', ...
    'EdgeColor', 'none');
hideObj(octagon3);

%% Plot and detect color for red hexagon
% Extract data from the red hexagon ply file, scale it to the environment and transform
% it to the desired position for the pickup instance. 
[f, v, data] = plyread('red_hexagon.ply', 'tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
hexagon_start_position = [0.25, -0.07, 0];
hexagon_scale_factor = 0.002;
hexagon_v_scaled = v * hexagon_scale_factor;
hexagon_v_transformation = hexagon_v_scaled + hexagon_start_position;
hexagon1 = trisurf(f, hexagon_v_transformation(:, 1), hexagon_v_transformation(:, 2), hexagon_v_transformation(:, 3), ...
    'FaceVertexCData', vertexColours, ...
    'FaceColor', 'interp', ...
    'EdgeColor', 'none');

% Extract data from the red hexagon ply file, scale it to the environment and transform
% it to the desired position for the end-effector instance. 
[f, v, data] = plyread('red_hexagon.ply', 'tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
hexagon_start_position2 = [0.28, 0.2, 0];
hexagon_scale_factor2 = 0.002;
hexagon_v_scaled2 = v * hexagon_scale_factor2;
hexagon_v_transformation2 = hexagon_v_scaled2 + hexagon_start_position2;
hexagon2 = trisurf(f, hexagon_v_transformation2(:, 1), hexagon_v_transformation2(:, 2), hexagon_v_transformation2(:, 3), ...
    'FaceVertexCData', vertexColours, ...
    'FaceColor', 'interp', ...
    'EdgeColor', 'none');
hideObj(hexagon2);

% Extract data from the red hexagon ply file, scale it to the environment and transform
% it to the desired position for the place instance. 
[f, v, data] = plyread('red_hexagon.ply', 'tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
hexagon_start_position3 = [0.28, 0.2, 0];
hexagon_scale_factor3 = 0.002;
hexagon_v_scaled3 = v * hexagon_scale_factor3;
hexagon_v_transformation3 = hexagon_v_scaled3 + hexagon_start_position3;
hexagon3 = trisurf(f, hexagon_v_transformation3(:, 1), hexagon_v_transformation3(:, 2), hexagon_v_transformation3(:, 3), ...
    'FaceVertexCData', vertexColours, ...
    'FaceColor', 'interp', ...
    'EdgeColor', 'none');
hideObj(hexagon3);

%% Movement Section for function use and Robot Control
% Variables to set the positions for the robot end effector to hit.
% Positions include the pick, place and plottable end effector poses.
% Includes transitional position between pick and place positions
square_position = [0.28, 0, 0.07];
octagon_position = [0.28, -0.16, 0.07];    % y-coordinate
hexagon_position = [0.2, -0.28, 0.07];    % y-coordinate
traj_position = [0.2, 0.15, 0.2];
place_position1 = [0, 0.2, 0.07];
place_position2 = [0.2, 0.2, 0.07];
place_position3 = [0.4, 0.1, 0.07];

% Calls relevant movement functions to iteratively pick up each object,
% hiding and showing each object at each point in the movement process to
% simualte the objects moving through the system. 
moveRobotToPosition(robot, square_position);
hideObj(square1);
moveRobotToPositionWithSquare(robot, traj_position);
moveRobotToPositionWithSquare(robot, place_position1);
showObj(square3);
moveRobotToPosition(robot, traj_position);
moveRobotToPosition(robot, octagon_position);
hideObj(octagon1);
moveRobotToPositionWithOctagon(robot, traj_position);
moveRobotToPositionWithOctagon(robot, place_position2);
showObj(octagon3);
moveRobotToPosition(robot, traj_position);
moveRobotToPosition(robot, hexagon_position);
hideObj(hexagon1);
moveRobotToPositionWithHexagon(robot, traj_position);
moveRobotToPositionWithHexagon(robot, place_position3);
showObj(hexagon3);
moveRobotToPosition(robot, traj_position);

%% Function to move the robot to a specified position
function moveRobotToPosition(robot, position)

    steps = 25;                        % Set the number of steps for the trajectory
    currentQ = robot.model.getpos();   % Get the current joint configuration of the robot

    % Calculate the target joint configuration using inverse kinematics
    % to reach the desired position
    targetQ = robot.model.ikcon(transl(position), currentQ);

    % Generate a joint trajectory from the current to target configuration
    traj = jtraj(currentQ, targetQ, steps);

    % Loop through each step in the trajectory
    for i = 1:steps
        robot.model.animate(traj(i, :));  % Move the robot to the next position in the trajectory
        pause(0.05);                      % Brief pause to control animation speed
    end

    pause(0.5);                           % Pause at the end to hold the final position
end

%% Function to move the robot to a specified position while moving an attached object (square)
function moveRobotToPositionWithSquare(robot, position)

    steps = 25;                          % Set the number of steps for the trajectory

    currentQ = robot.model.getpos();     % Get the current joint configuration of the robot

    % Calculate the target joint configuration to reach the desired position
    targetQ = robot.model.ikcon(transl(position), currentQ);

    % Generate a joint trajectory from the current to target configuration
    traj = jtraj(currentQ, targetQ, steps);

    % Get the initial pose of the end-effector
    endEffectorPose = robot.model.fkine(currentQ);

    % Determine the initial position for the object to be attached
    initial_object_position = endEffectorPose.t';
    initial_object_position = initial_object_position + [0, 0, -0.05]; % Adjust position slightly below end-effector

    % Load the 3D model of a green square object from a .ply file
    [f, v, data] = plyread('green_square.ply', 'tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    square_start_position2 = initial_object_position;
    square_scale_factor2 = 0.002;
    square_v_scaled2 = v * square_scale_factor2;
    square_v_transformation2 = square_v_scaled2 + square_start_position2;
    square_updated = trisurf(f, square_v_transformation2(:, 1), square_v_transformation2(:, 2), square_v_transformation2(:, 3), ...
        'FaceVertexCData', vertexColours, ...
        'FaceColor', 'interp', ...
        'EdgeColor', 'none');

    % Loop through each step in the trajectory
    for i = 1:steps
        robot.model.animate(traj(i, :));     % Move the robot to the next position in the trajectory

        % Update the object's position to follow the end-effector
        current_q = robot.model.getpos();
        endEffectorPose = robot.model.fkine(current_q);    % Calculate the current end-effector pose
        object_position = endEffectorPose.t';
        object_position = object_position + [0, 0, -0.05]; % Offset object position below end-effector

        % Hide the previous plot of the square to update its position
        hideObj(square_updated);

        % Load the 3D model and colors again for plotting
        [f, v, data] = plyread('green_square.ply', 'tri');
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Update position and scale for the square
        square_start_position2 = object_position;
        square_scale_factor2 = 0.002;
        square_v_scaled2 = v * square_scale_factor2;
        square_v_transformation2 = square_v_scaled2 + square_start_position2;

        % Plot the square in the new position, following the end-effector
        square_updated = trisurf(f, square_v_transformation2(:, 1), square_v_transformation2(:, 2), square_v_transformation2(:, 3), ...
            'FaceVertexCData', vertexColours, ...
            'FaceColor', 'interp', ...
            'EdgeColor', 'none');
        
        pause(0.05);                     % Brief pause for animation smoothness
        showObj(square_updated);         % Make the square visible in the updated position
    end

    pause(0.5);                         % Pause at the end to hold the final position
    hideObj(square_updated);            % Hide the object after the motion is completed
end

%% Function to move the robot to a specified position while moving an attached object (octagon)
function moveRobotToPositionWithOctagon(robot, position)
    steps = 25;                          % Set the number of steps for the trajectory

    currentQ = robot.model.getpos();     % Get the current joint configuration of the robot
    % Calculate the target joint configuration to reach the desired position
    targetQ = robot.model.ikcon(transl(position), currentQ);

    % Generate a joint trajectory from the current to target configuration
    traj = jtraj(currentQ, targetQ, steps);

    % Get the initial pose of the end-effector
    endEffectorPose = robot.model.fkine(currentQ);
    % Determine the initial position for the object to be attached
    initial_object_position = endEffectorPose.t';
    initial_object_position = initial_object_position + [0, 0, -0.05]; % Adjust position slightly below end-effector

    % Load the 3D model of a blue octagon object from a .ply file
    [f, v, data] = plyread('blue_octagon.ply', 'tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    octagon_start_position2 = initial_object_position;
    octagon_scale_factor2 = 0.002;
    octagon_v_scaled2 = v * octagon_scale_factor2;
    octagon_v_transformation2 = octagon_v_scaled2 + octagon_start_position2;
    octagon_updated = trisurf(f, octagon_v_transformation2(:, 1), octagon_v_transformation2(:, 2), octagon_v_transformation2(:, 3), ...
        'FaceVertexCData', vertexColours, ...
        'FaceColor', 'interp', ...
        'EdgeColor', 'none');

    % Loop through each step in the trajectory
    for i = 1:steps
        robot.model.animate(traj(i, :));     % Move the robot to the next position in the trajectory

        % Update the object's position to follow the end-effector
        current_q = robot.model.getpos();
        endEffectorPose = robot.model.fkine(current_q); % Calculate the current end-effector pose
        object_position = endEffectorPose.t';
        object_position = object_position + [0, 0, -0.05]; % Offset object position below end-effector

        % Hide the previous plot of the octagon to update its position
        hideObj(octagon_updated);

        % Load the 3D model and colors again for plotting
        [f, v, data] = plyread('blue_octagon.ply', 'tri');
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Update position and scale for the octagon
        octagon_start_position2 = object_position;
        octagon_scale_factor2 = 0.002;
        octagon_v_scaled2 = v * octagon_scale_factor2;
        octagon_v_transformation2 = octagon_v_scaled2 + octagon_start_position2;

        % Plot the octagon in the new position, following the end-effector
        octagon_updated = trisurf(f, octagon_v_transformation2(:, 1), octagon_v_transformation2(:, 2), octagon_v_transformation2(:, 3), ...
            'FaceVertexCData', vertexColours, ...
            'FaceColor', 'interp', ...
            'EdgeColor', 'none');
        
        pause(0.05);                    % Brief pause for animation smoothness
        showObj(octagon_updated);       % Make the octagon visible in the updated position
    end

    pause(0.5);                         % Pause at the end to hold the final position
    hideObj(octagon_updated);           % Hide the object after the motion is completed
end

%% Function to move the robot to a specified position while moving an attached hexagon object
function moveRobotToPositionWithHexagon(robot, position)

    steps = 25;                          % Set the number of steps for the trajectory

    currentQ = robot.model.getpos();     % Get the current joint configuration of the robot

    % Calculate the target joint configuration to reach the desired position
    targetQ = robot.model.ikcon(transl(position), currentQ);

    % Generate a joint trajectory from the current to target configuration
    traj = jtraj(currentQ, targetQ, steps);

    % Get the initial pose of the end-effector
    endEffectorPose = robot.model.fkine(currentQ);

    % Determine the initial position for the object to be attached
    initial_object_position = endEffectorPose.t';
    initial_object_position = initial_object_position + [0, 0, -0.05]; % Adjust position slightly below end-effector

    % Load the 3D model of a red hexagon object from a .ply file
    [f, v, data] = plyread('red_hexagon.ply', 'tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    hexagon_start_position2 = initial_object_position;
    hexagon_scale_factor2 = 0.002;
    hexagon_v_scaled2 = v * hexagon_scale_factor2;
    hexagon_v_transformation2 = hexagon_v_scaled2 + hexagon_start_position2;
    hexagon_updated = trisurf(f, hexagon_v_transformation2(:, 1), hexagon_v_transformation2(:, 2), hexagon_v_transformation2(:, 3), ...
        'FaceVertexCData', vertexColours, ...
        'FaceColor', 'interp', ...
        'EdgeColor', 'none');

    % Loop through each step in the trajectory
    for i = 1:steps
        robot.model.animate(traj(i, :));     % Move the robot to the next position in the trajectory

        % Update the object's position to follow the end-effector
        current_q = robot.model.getpos();
        endEffectorPose = robot.model.fkine(current_q); % Calculate the current end-effector pose
        object_position = endEffectorPose.t';
        object_position = object_position + [0, 0, -0.05]; % Offset object position below end-effector

        % Hide the previous plot of the hexagon to update its position
        hideObj(hexagon_updated);

        % Load the 3D model and colors again for plotting
        [f, v, data] = plyread('red_hexagon.ply', 'tri');
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Update position and scale for the hexagon
        hexagon_start_position2 = object_position;
        hexagon_scale_factor2 = 0.002;
        hexagon_v_scaled2 = v * hexagon_scale_factor2;
        hexagon_v_transformation2 = hexagon_v_scaled2 + hexagon_start_position2;

        % Plot the hexagon in the new position, following the end-effector
        hexagon_updated = trisurf(f, hexagon_v_transformation2(:, 1), hexagon_v_transformation2(:, 2), hexagon_v_transformation2(:, 3), ...
            'FaceVertexCData', vertexColours, ...
            'FaceColor', 'interp', ...
            'EdgeColor', 'none');
        
        pause(0.05);                    % Brief pause for animation smoothness
        showObj(hexagon_updated);       % Make the hexagon visible in the updated position
    end

    pause(0.5);                         % Pause at the end to hold the final position
    hideObj(hexagon_updated);           % Hide the object after the motion is completed
end

%% Function to hid objects
function hideObj(obj)
    obj.Visible = 'off';
end

%% Function to show objects
function showObj(obj)
    obj.Visible = 'on';
end


