% %% Workspace Setup Function
%     clf;
%     clc;
%     % Create and plot robot
%     robot = DobotMagician();
%     q0 = [0, pi/6, pi/4, pi/2, 0];
%     workspace = [-0.5, 0.5, -0.5, 0.5, 0, 0.5];
%     scale = 0.5;
%     robot.model.plot(q0,'workspace',workspace,'scale',scale);
%     axis(workspace);
%     hold on;
% 
%     % Plot for green square
%     [f, v, data] = plyread('green_square.ply', 'tri');
%     vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%     square_start_position = [0.25, 0, 0];
%     square_scale_factor = 0.002;
%     square_v_scaled = v * square_scale_factor;
%     square_v_transformation = square_v_scaled + square_start_position;
%     square1 = trisurf(f, square_v_transformation(:, 1), square_v_transformation(:, 2), square_v_transformation(:, 3), ...
%         'FaceVertexCData', vertexColours, ...
%         'FaceColor', 'interp', ...
%         'EdgeColor', 'none');
% 
%     % Plot for blue triangle
%     [f, v, data] = plyread('blue_octagon.ply', 'tri');
%     vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%     triangle_start_position = [0.25, 0.07, 0];
%     triangle_scale_factor = 0.002;
%     triangle_v_scaled = v * triangle_scale_factor;
%     triangle_v_transformation = triangle_v_scaled + triangle_start_position;
%     triangle1 = trisurf(f,  triangle_v_transformation(:, 1),  triangle_v_transformation(:, 2),  triangle_v_transformation(:, 3), ...
%         'FaceVertexCData', vertexColours, ...
%         'FaceColor', 'interp', ...
%         'EdgeColor', 'none');
% 
%     % Plot for red hexagon
%     [f, v, data] = plyread('red_hexagon.ply', 'tri');
%     vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%     start_position = [0.25, -0.07, 0];
%     scale_factor = 0.002;
%     v_scaled = v * scale_factor;
%     v_transformation = v_scaled + start_position;
%     hexagon1 = trisurf(f, v_transformation(:, 1), v_transformation(:, 2), v_transformation(:, 3), ...
%         'FaceVertexCData', vertexColours, ...
%         'FaceColor', 'interp', ...
%         'EdgeColor', 'none');
% 


%% Workspace Setup Function
clf;
clc;
% Create and plot robot
robot = DobotMagician();
q0 = [0, pi/6, pi/4, pi/2, 0];
workspace = [-1, 1, -1, 1, -1, 1];
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
detectedColor_square = detectObjectColor(vertexColours);
disp(['Detected Color for Square: ', detectedColor_square]);

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
detectedColor_octagon = detectObjectColor(vertexColours);
disp(['Detected Color for Octagon: ', detectedColor_octagon]);

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
detectedColor_hexagon = detectObjectColor(vertexColours);
disp(['Detected Color for Hexagon: ', detectedColor_hexagon]);

%% After plotting the objects (green square, blue octagon, red hexagon)
detectAndPlaceShape(robot, square_v_transformation, vertexColours, square_start_position);
detectAndPlaceShape(robot, triangle_v_transformation, vertexColours, triangle_start_position);
detectAndPlaceShape(robot, v_transformation, vertexColours, start_position);
