import cv2
import numpy as np
import glob

# Define the dimensions of checkerboard (number of inner corners)
CHECKERBOARD = (6, 9)

# Stop criteria for subpixel refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Vectors to store vectors of 3D points and 2D points
threedpoints = []
twodpoints = []

# Prepare 3D points real world coordinates (0,0,0), (1,0,0), (2,0,0) ...
objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Grab all jpg files
images = glob.glob('*.jpg')

if not images:
    print("Error: No .jpg files found in the directory!")
    exit()

# Variable to store the shape of the images
gray_shape = None

for filename in images:
    image = cv2.imread(filename)
    if image is None:
        continue
        
    grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_shape = grayColor.shape[::-1] # Store (width, height)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(
                    grayColor, CHECKERBOARD, 
                    cv2.CALIB_CB_ADAPTIVE_THRESH + 
                    cv2.CALIB_CB_FAST_CHECK + 
                    cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret == True:
        threedpoints.append(objectp3d)

        # Refining pixel coordinates
        corners2 = cv2.cornerSubPix(
            grayColor, corners, (11, 11), (-1, -1), criteria)

        twodpoints.append(corners2)

        # Draw and display the corners
        image = cv2.drawChessboardCorners(image, CHECKERBOARD, corners2, ret)

    cv2.imshow('Calibration Progress', image)
    cv2.waitKey(100) # Show for 100ms instead of waiting for keypress

cv2.destroyAllWindows()

# Check if we found enough points to calibrate
if len(threedpoints) > 0:
    print(f"Calibrating using {len(threedpoints)} images...")
    
    # Perform camera calibration
    ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
        threedpoints, twodpoints, gray_shape, None, None)

    # Displaying required output
    print("\n--- Calibration Results ---")
    print("Camera matrix (Intrinsic parameters):")
    print(matrix)

    print("\nDistortion coefficients:")
    print(distortion)
    
    # Optional: Calculate Re-projection Error to check accuracy
    mean_error = 0
    for i in range(len(threedpoints)):
        imgpoints2, _ = cv2.projectPoints(threedpoints[i], r_vecs[i], t_vecs[i], matrix, distortion)
        error = cv2.norm(twodpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print(f"\nTotal Re-projection Error: {mean_error/len(threedpoints)}")

else:
    print("Calibration failed: No checkerboard corners were detected in any images.")