% Helper function to move the robot to a specified position
function moveRobotToPosition(robot, position)
    % Custom code to move the robot model to the desired position
    % In your actual setup, you would issue the commands to control the real robot
    
    % Here we are simulating the robot motion by setting the end-effector position
    currentQ = robot.model.getpos();  % Get current joint angles
    targetQ = robot.model.ikcon(transl(position), currentQ);  % Compute inverse kinematics
    robot.model.animate(targetQ);  % Move the robot to the target joint angles
    pause(1);  % Pause for visualization (adjust for actual robot)
end