% Initialize Serial Port
dobot = serialport("COM3", 115200);
configureTerminator(dobot, "CR/LF");

% Flush input/output buffers
flush(dobot); 

% Send PING Command
try
    writeline(dobot, "PING");  % Try sending PING command
    pause(1);  % Allow time for a response

    if dobot.NumBytesAvailable > 0
        response = readline(dobot);
        disp("Response from Dobot: " + response);
    else
        disp("No response received from Dobot.");
    end
catch ME
    disp("Error during communication:");
    disp(ME.message);
end

% Close the serial port when done
try
    clear dobot;  % Close serial port object
    disp("Serial port closed successfully.");
catch ME
    disp("Error closing serial port:");
    disp(ME.message);
end
