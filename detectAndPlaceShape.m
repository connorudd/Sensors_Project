function detectAndPlaceShape(robot, objectVertices, vertexColours, objectStartPosition)
    % Detect color of the shape
    detectedColor = detectObjectColor(vertexColours);
    disp(['Detected Color: ', detectedColor]);

    % Define plate positions for red, green, and blue
    platePositions = struct('Red', [0.3, -0.2, 0.01], ...
                            'Green', [0.3, 0.0, 0.01], ...
                            'Blue', [0.3, 0.2, 0.01]);

    % Determine the target position based on the detected color
    if isfield(platePositions, detectedColor)
        targetPosition = platePositions.(detectedColor);
    else
        disp('No corresponding plate found for detected color.');
        return;
    end

    % Move robot to the shape's position (above the object)
    moveRobotToPosition(robot, objectStartPosition + [0, 0, 0.05]); % Approach the object from above

    % Move robot down to pick up the object
    moveRobotToPosition(robot, objectStartPosition);

    % Simulate the pick-up (customize this for gripper or end-effector controls)
    disp('Picking up the object...');

    % Move the object to the target plate
    moveRobotToPosition(robot, targetPosition + [0, 0, 0.05]); % Approach the plate from above
    moveRobotToPosition(robot, targetPosition);  % Place the object

    % Simulate the release (customize this for gripper or end-effector controls)
    disp(['Placing object on the ', detectedColor, ' plate.']);
end

