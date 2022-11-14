#!/usr/bin/env python3

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = np.arctan(-0.085)
beta = 733.0
tx = 0.244
ty = 0.18


# Function that converts image coord to world coord
def IMG2W(col, row):
    Cencol_y = col - 320
    Cenrow_x = row - 240 
    ca_x = Cenrow_x /beta +tx
    ca_y = Cencol_y /beta +ty
    w_y = (ca_x - ca_y * np.tan(theta))*np.sin(theta) + (ca_y / np.cos(theta))

    w_x = (ca_x - ca_y * np.tan(theta))*np.cos(theta)
    return (w_x,w_y)


# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False
    params.minThreshold = 10
    params.maxThreshold = 200

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 1000

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.5
    params.maxCircularity = 1

    # Filter by Inerita
    params.filterByInertia = False
    params.minInertiaRatio = 0.01
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.95

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    #lower = (110,50,50)     # blue lower
    #upper = (130,255,255)   # blue upper
    if(color == "green"):
        lower = (50,50,20)     # green lower
        upper = (70,255,255)   # green upper

    #lower = (0,80,80)     # green and orange lower
    #upper = (80,255,255)   # green and orange upper
    if(color == "orange"):
        lower = (0,80,80)     # orange lower
        upper = (25,255,255)   # orange upper

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
        # print((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints,np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
