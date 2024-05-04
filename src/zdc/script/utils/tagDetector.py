import cv2
from pathlib import Path
import numpy as np
import rospy

from . import make_detector

default_imgpath = Path(__file__).parent.parent / "img"

# RECOGNITION

def recognize_arucos(img, wimg = False, detector : cv2.aruco.ArucoDetector = None) :
    """Recognizes the arucos in the image and displays them if wimg is set to True
    
    Args:
        img (np.array): Image to process
        wimg (bool, optional): Whether to display the image with the arucos. Defaults to False.
        
    Returns:
        tuple: corners, ids, rejected, the detected arucos' corners, ids and rejected arucos
    """

    if detector is None :
        detector = make_detector.make_detector()
        
    (corners, ids, rejected) = detector.detectMarkers(img)
    # rospy.loginfo(f"Detected arucos : {ids}")

    if wimg : #if display img
        try :
            img_copy = img.copy()
            # Draw the detected arucos
            for i in range(len(ids)) :
                x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4
                y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4
                cv2.drawMarker(img_copy, (int(x), int(y)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2) # draw the center of the aruco
            # Draw the rejected arucos
            for i in range(len(rejected)) :
                x = (rejected[i][0][0][0] + rejected[i][0][1][0] + rejected[i][0][2][0] + rejected[i][0][3][0]) / 4
                y = (rejected[i][0][0][1] + rejected[i][0][1][1] + rejected[i][0][2][1] + rejected[i][0][3][1]) / 4
                cv2.drawMarker(img_copy, (int(x), int(y)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        except Exception as e :
            rospy.logerr(f"Error while displaying the image : {e}")

    return corners, ids, rejected

# DISPLAY

def show_real_pos_img(img : np.ndarray, blue_pos : tuple, yellow_pos : tuple) :
    """Displays the image with the calculated position of the robot
    Args:
        img : the image
        x : the x position of the robot
        y : the y position of the robot
    """
    
    img_copy = img.copy()
    width = img.shape[1]
    height = img.shape[0]
    x_blue, y_blue, alpha_blue = blue_pos
    x_yellow, y_yellow, alpha_yellow = yellow_pos
    
    x_blue = width / 4 + x_blue * width / 600
    y_blue = height - (height / 4 + y_blue * height / 400)
    x_yellow = width / 4 + x_yellow * width / 600
    y_yellow = height - (height / 4 + y_yellow * height / 400)

    # draw position
    cv2.drawMarker(img_copy, (int(x_blue), int(y_blue)), (0, 0, 255), cv2.MARKER_CROSS, 20, 10)
    cv2.drawMarker(img_copy, (int(x_yellow), int(y_yellow)), (0, 255, 255), cv2.MARKER_CROSS, 20, 10)
    #draw rotation
    x2_blue = x_blue + 100 * np.cos(alpha_blue)
    y2_blue = y_blue + 100 * np.sin(alpha_blue)
    x2_yellow = x_yellow + 100 * np.cos(alpha_yellow)
    y2_yellow = y_yellow + 100 * np.sin(alpha_yellow)
    cv2.line(img_copy, (int(x_blue), int(y_blue)), (int(x2_blue), int(y2_blue)), (0, 0, 255), 10)
    cv2.line(img_copy, (int(x_yellow), int(y_yellow)), (int(x2_yellow), int(y2_yellow)), (0, 255, 255), 10)
        
    img_copy = cv2.resize(img_copy, (int(img.shape[1] / 4), int(img.shape[0] / 4)))

    cv2.imshow("Real position", img_copy)
    cv2.waitKey(10)