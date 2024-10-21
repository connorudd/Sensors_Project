function detectedColor = detectObjectColor(vertexColors)
    % This function detects the dominant color (red, green, or blue) 
    % based on the vertex colors from the PLY file.
    
    % Extract mean color values
    avgRed = mean(vertexColors(:,1));
    avgGreen = mean(vertexColors(:,2));
    avgBlue = mean(vertexColors(:,3));
    
    % Compare the average color intensities
    if avgRed > avgGreen && avgRed > avgBlue
        detectedColor = 'Red';
    elseif avgGreen > avgRed && avgGreen > avgBlue
        detectedColor = 'Green';
    elseif avgBlue > avgRed && avgBlue > avgGreen
        detectedColor = 'Blue';
    else
        detectedColor = 'Unknown';
    end
end
