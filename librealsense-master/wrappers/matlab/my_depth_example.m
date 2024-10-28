function my_depth_example()
    % Make Pipeline object to manage streaming
    disp('Creating pipeline object...');
    pipe = realsense.pipeline();
    
    % Make Colorizer object to prettify depth output
    disp('Creating colorizer object...');
    colorizer = realsense.colorizer();

    % Start streaming on an arbitrary camera with default settings
    disp('Starting camera streaming...');
    profile = pipe.start();

    % Get streaming device's name
    disp('Retrieving camera device information...');
    dev = profile.get_device();
    name = dev.get_info(realsense.camera_info.name);
    disp(['Camera name: ', name]);

    % Get frames. We discard the first few to allow
    % the camera time to settle
    disp('Capturing initial frames to allow camera to stabilize...');
    for i = 1:5
        fs = pipe.wait_for_frames();
        disp(['Frame ', num2str(i), ' captured.']);
    end
    
    % Stop streaming
    disp('Stopping camera streaming...');
    pipe.stop();

    % Select depth frame
    disp('Selecting depth frame...');
    depth = fs.get_depth_frame();
    
    % Verify depth frame is valid
    if ~depth.is_valid()
        error('Depth frame is invalid. Please check the camera connection and setup.');
    else
        disp('Depth frame is valid.');
    end

    % Colorize depth frame
    disp('Colorizing depth frame...');
    color = colorizer.colorize(depth);

    % Get actual data and convert into a format imshow can use
    % (Color data arrives as [R, G, B, R, G, B, ...] vector)
    disp('Converting colorized frame data to image format...');
    data = color.get_data();
    img = permute(reshape(data', [3, color.get_width(), color.get_height()]), [3, 2, 1]);

    % Display image
    disp('Displaying colorized depth frame image...');
    imshow(img);
    title(sprintf("Colorized depth frame from %s", name));
    disp('Image displayed successfully.');
end
