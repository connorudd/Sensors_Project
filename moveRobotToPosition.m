% Helper function to move the robot to a specified position
function moveRobotToPosition(robot, position)
    % Define the offset between the robot's base frame and the world frame
    offset = [0.05, 0.02, 0];  % Adjust these values to calibrate the position

    % Apply the offset to the position
    adjustedPosition = position + offset;

    % Compute the inverse kinematics for the adjusted position
    currentQ = robot.model.getpos();  % Get current joint angles
    targetQ = robot.model.ikcon(transl(adjustedPosition), currentQ);  % Compute inverse kinematics
    robot.model.animate(targetQ);  % Move the robot to the target joint angles
    pause(1);  % Pause for visualization (adjust for actual robot)
end
