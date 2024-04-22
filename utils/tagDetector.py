import cv2
from pathlib import Path
import numpy as np
import logging
import yaml

from . import make_detector

default_imgpath = Path(__file__).parent.parent / "img"

# Logger
logging.basicConfig(level = logging.INFO)
logger = logging.getLogger(__name__)
with open(str(Path(__file__).parent.parent / "config" / "board.yaml"), "r") as f :
    config = yaml.safe_load(f)
    logger.setLevel(config["logger"])

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
    print(ids)
    logger.debug("Detected arucos : {}".format(ids))

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
            cv2.imwrite(str(default_imgpath / "detected_arucos.jpg"), img_copy)
        except Exception as e :
            logger.error("Error while displaying the image with the arucos : {}".format(e))

    return corners, ids, rejected

# DISPLAY

def show_real_pos_img(img : np.ndarray, x : float, y : float, alpha : float = 0.) :
    """Displays the image with the calculated position of the robot
    Args:
        img : the image
        x : the x position of the robot
        y : the y position of the robot
    """
    
    img_copy = img.copy()
    width = img.shape[1]
    height = img.shape[0]
    
    x = width / 4 + x * width / 600
    y = height - (height / 4 + y * height / 400)
    # draw position
    cv2.drawMarker(img_copy, (int(x), int(y)), (0, 255, 0), cv2.MARKER_CROSS, 20, 10)
    #draw rotation
    x2 = x + 100 * np.cos(alpha)
    y2 = y + 100 * np.sin(alpha)
    cv2.line(img_copy, (int(x), int(y)), (int(x2), int(y2)), (0, 255, 0), 10)
    
    img_copy = cv2.resize(img_copy, (int(img.shape[1] / 4), int(img.shape[0] / 4)))

    cv2.imshow("Real position", img_copy)
    cv2.waitKey(10)