'''
     This file is part of telemetry_thermal_versions.

    telemetry_thermal_versions is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

    telemetry_thermal_versions is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    GNU General Public License: <https://www.gnu.org/licenses/>. 
'''

import cv2
import numpy as np
import io
import sys  # Import the sys module

# Check if a filename was provided
if len(sys.argv) < 2:
    print("Usage: python3 thermal_snapshot.py FILENAME.png")
    sys.exit(1)  # Exit the script if no argument is given

filename = sys.argv[1]  # Get the filename from the command line arguments

# Initialize video capture
dev = 0  # Default video device number
cap = cv2.VideoCapture('/dev/video' + str(dev), cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)

# Capture a single frame
ret, frame = cap.read()
if ret:
    # Process the frame
    imdata, _ = np.array_split(frame, 2)
    bgr = cv2.cvtColor(imdata, cv2.COLOR_YUV2BGR_YUYV)
    heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_JET)

    # Save the image using the provided filename
    cv2.imwrite(filename, heatmap)  # Use the passed-in filename

# Release resources
cap.release()