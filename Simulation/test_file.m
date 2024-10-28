%% Workspace Setup Function
clf;
clc;
% Create and plot robot
robot = DobotMagician();
q0 = [0, pi/6, pi/4, pi/2, 0];
workspace = [-0.4, 0.4, -0.4, 0.4, 0, 0.4];
scale = 0.5;
robot.model.plot(q0,'workspace',workspace,'scale',scale);
axis(workspace);
hold on;

%% Plot and detect color for green square
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

%%
square_position = [0.28, 0, 0.07];
octagon_position = [0.28, -0.16, 0.07];    % y-coordinate
hexagon_position = [0.2, -0.28, 0.07];    % y-coordinate
traj_position = [0.2, 0.15, 0.2];
place_position1 = [0, 0.2, 0.07];
place_position2 = [0.2, 0.2, 0.07];
place_position3 = [0.4, 0.1, 0.07];

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

%% Function to move the robot from to position
function moveRobotToPosition(robot, position)
    steps = 25;
    currentQ = robot.model.getpos();
    targetQ = robot.model.ikcon(transl(position), currentQ);
    traj = jtraj(currentQ, targetQ, steps);
    for i = 1:steps
        robot.model.animate(traj(i, :));
        pause(0.05);
    end
    pause(0.5);
end

%% Function to move the robot from to position
function moveRobotToPositionWithSquare(robot, position)
    steps = 25;
    currentQ = robot.model.getpos();
    targetQ = robot.model.ikcon(transl(position), currentQ);
    traj = jtraj(currentQ, targetQ, steps);
    endEffectorPose = robot.model.fkine(currentQ);
    initial_object_position = endEffectorPose.t';
    initial_object_position = initial_object_position + [0, 0, -0.05];
    
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

    for i = 1:steps
        robot.model.animate(traj(i, :));

        current_q = robot.model.getpos();
        endEffectorPose = robot.model.fkine(current_q);
        object_position = endEffectorPose.t';
        object_position = object_position + [0, 0, -0.05];
        hideObj(square_updated);
        [f, v, data] = plyread('green_square.ply', 'tri');
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        square_start_position2 = object_position;
        square_scale_factor2 = 0.002;
        square_v_scaled2 = v * square_scale_factor2;
        square_v_transformation2 = square_v_scaled2 + square_start_position2;
        square_updated = trisurf(f, square_v_transformation2(:, 1), square_v_transformation2(:, 2), square_v_transformation2(:, 3), ...
            'FaceVertexCData', vertexColours, ...
            'FaceColor', 'interp', ...
            'EdgeColor', 'none');
        pause(0.05);
        showObj(square_updated)
    end
    pause(0.5);
    hideObj(square_updated);
end

%% Function to move the robot from to position
function moveRobotToPositionWithOctagon(robot, position)
    steps = 25;
    currentQ = robot.model.getpos();
    targetQ = robot.model.ikcon(transl(position), currentQ);
    traj = jtraj(currentQ, targetQ, steps);
    endEffectorPose = robot.model.fkine(currentQ);
    initial_object_position = endEffectorPose.t';
    initial_object_position = initial_object_position + [0, 0, -0.05];
    
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

    for i = 1:steps
        robot.model.animate(traj(i, :));

        current_q = robot.model.getpos();
        endEffectorPose = robot.model.fkine(current_q);
        object_position = endEffectorPose.t';
        object_position = object_position + [0, 0, -0.05];
        hideObj(octagon_updated);
        [f, v, data] = plyread('blue_octagon.ply', 'tri');
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        octagon_start_position2 = object_position;
        octagon_scale_factor2 = 0.002;
        octagon_v_scaled2 = v * octagon_scale_factor2;
        octagon_v_transformation2 = octagon_v_scaled2 + octagon_start_position2;
        octagon_updated = trisurf(f, octagon_v_transformation2(:, 1), octagon_v_transformation2(:, 2), octagon_v_transformation2(:, 3), ...
            'FaceVertexCData', vertexColours, ...
            'FaceColor', 'interp', ...
            'EdgeColor', 'none');
        pause(0.05);
        showObj(octagon_updated)
    end
    pause(0.5);
    hideObj(octagon_updated);
end

%% Function to move the robot from to position
function moveRobotToPositionWithHexagon(robot, position)
    steps = 25;
    currentQ = robot.model.getpos();
    targetQ = robot.model.ikcon(transl(position), currentQ);
    traj = jtraj(currentQ, targetQ, steps);
    endEffectorPose = robot.model.fkine(currentQ);
    initial_object_position = endEffectorPose.t';
    initial_object_position = initial_object_position + [0, 0, -0.05];
    
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

    for i = 1:steps
        robot.model.animate(traj(i, :));

        current_q = robot.model.getpos();
        endEffectorPose = robot.model.fkine(current_q);
        object_position = endEffectorPose.t';
        object_position = object_position + [0, 0, -0.05];
        hideObj(hexagon_updated);
        [f, v, data] = plyread('red_hexagon.ply', 'tri');
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        hexagon_start_position2 = object_position;
        hexagon_scale_factor2 = 0.002;
        hexagon_v_scaled2 = v * hexagon_scale_factor2;
        hexagon_v_transformation2 = hexagon_v_scaled2 + hexagon_start_position2;
        hexagon_updated = trisurf(f, hexagon_v_transformation2(:, 1), hexagon_v_transformation2(:, 2), hexagon_v_transformation2(:, 3), ...
            'FaceVertexCData', vertexColours, ...
            'FaceColor', 'interp', ...
            'EdgeColor', 'none');
        pause(0.05);
        showObj(hexagon_updated)
    end
    pause(0.5);
    hideObj(hexagon_updated);
end
%% Function to hid objects
function hideObj(obj)
    obj.Visible = 'off';
end

%% Function to show objects
function showObj(obj)
    obj.Visible = 'on';
end



