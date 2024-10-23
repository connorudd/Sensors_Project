% Initialize Serial Port
dobot = serialport("COM3", 115200);
configureTerminator(dobot, "CR/LF");

% Send PING Command
try
    writeline(dobot, "PING");  % or "VER" to check version
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
