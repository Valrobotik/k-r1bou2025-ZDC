#!/usr/bin/env python3
"""Determines the position of a robot on the board from a flattened image of the board."""

from pathlib import Path
import numpy as np
import yaml
import cv2
import time
import rospy
from std_msgs.msg import Float32MultiArray

from utils import flattenimg as flt, tagDetector as tagd, reset, make_detector

# Logger
with open(str(Path(__file__).parent / "config" / "board.yaml"), "r") as f :
    config = yaml.safe_load(f)
    pts1 = np.float32(config["img"]["perspective"]["pts1"])
    pts2 = np.float32(config["img"]["perspective"]["pts2"])

def get_rotation(img, id_range_blue : range, id_range_yellow : range, detector : cv2.aruco.ArucoDetector = None) :
    """Returns the rotation of the aruco with the given id. This assumes that the image was flattened.
    
    Args:
        img (np.array): Image to process
        id_range_blue (range): The range of the blue team's aruco ids
        id_range_yellow (range): The range of the yellow team's aruco ids
        detector (cv2.aruco.ArucoDetector, optional): The aruco detector. Defaults to None, but should be set for better performance.
    Returns:
        rotation (float): the rotation of the aruco, in radians
    """
    if detector is None :
        detector = make_detector.make_detector()

    rotation_blue, rotation_yellow = 0., 0.
    
    (corners, ids, _) = detector.detectMarkers(img)
    
    if ids is None :
        rospy.logerr("(CAMERA POLE) No aruco found while calculating the rotation")
        return 0.
    
    for i in range(len(ids)) :
        if ids[i] in id_range_blue or ids[i] in id_range_yellow :
            x1 = corners[i][0][3][0]
            y1 = corners[i][0][3][1]
            x2 = corners[i][0][0][0]
            y2 = corners[i][0][0][1]
            
            if ids[i] in id_range_blue :
                rotation_blue = np.arctan2((y2 - y1), (x2 - x1))
            else :
                rotation_yellow = np.arctan2((y2 - y1), (x2 - x1))

    return rotation_blue, rotation_yellow

def calc_real_pos(flat_img : np.ndarray, pole_decal : int, pole_height : int, ar_height_blue : int, ar_height_yellow : int, id_range_blue : range, id_range_yellow : range, detector : cv2.aruco.ArucoDetector = None) :
    """Calculates the real position of the robot from the flattened image, according to the position of the pole
    params :
        flat_img : the flattened image
        pole_decal : the distance between the pole and the center of the board : negative value -> left, positive value -> right
        pole_height : the height of the pole
        ar_height_blue : the height of the blue team aruco
        ar_height_yellow : the height of the yellow team aruco
        id_range_blue : the range of the blue team aruco ids
        id_range_yellow : the range of the yellow team aruco ids
        detector : the aruco detector
    """
    
    #Load the image and the arucos
    img_height = len(flat_img) // 2
    img_width = len(flat_img[0]) // 2

    x_blue, y_blue, x_yellow, y_yellow = -1, -1, -1, -1
    corners, ids, _ = tagd.recognize_arucos(flat_img, False, detector)
    
    if ids is None :
        rospy.logerr("(CAMERA POLE) No aruco found while calculating the position")
        return -1, -1
    
    #Process the position of the aruco
    for i in range(len(ids)) :
        if ids[i][0] in id_range_blue or ids[i][0] in id_range_yellow :
            x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4
            y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4
            
            #Conversion from pixels to meters
            x = 300 * (x - img_width / 2) / img_width
            y = 200 * (y - img_height / 2) / img_height

            #Calculate the distance from the aruco
            pos_camera = 150 + pole_decal
            dist_x = abs(x - pos_camera)

            #Invert y axis
            y = 200 - y

            #Angles
            alphax = np.arctan(dist_x / pole_height)
            alphay = np.arctan(y / pole_height)
            
            #Real position
            if ids[i][0] in id_range_blue :
                lx = (pole_height - ar_height_blue) * np.tan(alphax)
                ly = (pole_height - ar_height_blue) * np.tan(alphay)
                posx = 150+lx if x > pos_camera else 150-lx
                x_blue, y_blue = posx, ly

            else :
                lx = (pole_height - ar_height_yellow) * np.tan(alphax)
                ly = (pole_height - ar_height_yellow) * np.tan(alphay)
                posx = 150+lx if x > pos_camera else 150-lx
                x_yellow, y_yellow = posx, ly

    if x_blue == -1 and y_blue == -1 :
        rospy.logerr(f"(CAMERA POLE) Blue Team Aruco not found while calculating the position")
    if x_yellow == -1 and y_yellow == -1 :
        rospy.logerr(f"(CAMERA POLE) Yellow Team Aruco not found while calculating the position")

    return (x_blue, y_blue), (x_yellow, y_yellow)

def calc_real_pos_and_rot(img : np.ndarray, pole_decal : int, pole_height : int, ar_height_blue : int, ar_height_yellow : int, id_range_blue : range, id_range_yellow : range, detector : cv2.aruco.ArucoDetector = None, show = False) :
    """Calculates the real position of the robot from the camera.
    
    Args:
        img (np.ndarray): The image taken by the camera
        cam (cv2.VideoCapture): The camera
        config (dict): The config dictionary
        detector (cv2.aruco.ArucoDetector, optional): The aruco detector. Defaults to None, but should be set for better performance.
        
    Returns:
        tuple: the x, y and alpha values of the robot
    """
    global pts1, pts2
    
    unwarped_img = flt.unwarp_img(img, detector, (pts1, pts2), False)
    
    # Get the position of the robot
    (x_blue, y_blue), (x_yellow, y_yellow) = calc_real_pos(unwarped_img, pole_decal, pole_height, ar_height_blue, ar_height_yellow, id_range_blue, id_range_yellow, detector)
    
    # Get the rotation of the robot
    alpha_blue, alpha_yellow = get_rotation(unwarped_img, id_range_blue, id_range_yellow, detector)
    
    rospy.loginfo(f"Blue robot : {x_blue}, {y_blue}, {alpha_blue}")
    rospy.loginfo(f"Yellow robot : {x_yellow}, {y_yellow}, {alpha_yellow}")
    
    if show :
        tagd.show_real_pos_img(unwarped_img, (x_blue, y_blue, alpha_blue), (x_yellow, y_yellow, alpha_yellow))

    return (x_blue, y_blue, alpha_blue), (x_yellow, y_yellow, alpha_yellow)

def main() :
    """The main function of the module, calculates the real position of the robot from the flattened image of the board.
    Do not forget to modify the config file for the robot's aruco id and the pole's position
    Args:
        robot_color (int, optional): The color of the robot. Defaults to 0. 0 for blue, 1 for yellow.
    """
    global pts1, pts2

    reset.reset_perspective()
    rospy.init_node("camera_pole")
    pos_rot_pub = rospy.Publisher("camera", Float32MultiArray, queue_size=1)
    rospy.loginfo("[START] Camera_pole node has started.")
    
    # Load config file
    config = yaml.safe_load(open(str(Path(__file__).parent / "config" / "board.yaml")))
    delay_reset = config["delay_reset"]
    ar_height_blue = config["robots"]["blue_ar_height"]
    ar_height_yellow = config["robots"]["yellow_ar_height"]
    pole_decal = config["pole"]["decal"]
    pole_height = config["pole"]["height"]
    id_range_blue = range(config["robots"]["blue_id_range"][0], config["robots"]["blue_id_range"][1])
    id_range_yellow = range(config["robots"]["yellow_id_range"][0], config["robots"]["yellow_id_range"][1])
    ar_height_blue = config["robots"]["blue_ar_height"]
    ar_height_yellow = config["robots"]["yellow_ar_height"]
    cam_id = config["cam_id"]
    display = True if config["display"] == 1 else False
    
    detector = make_detector.make_detector()
    
    cam = cv2.VideoCapture(cam_id)
    last_reset = time.time()

    while not cam.isOpened() or not cam.read()[0]:
        rospy.logerr("(CAMERA POLE) Error while opening the camera")
        rospy.sleep(1)
        cam = cv2.VideoCapture(cam_id)

    while not flt.edgeArucoDetection(cam.read()[1], detector) :
        rospy.loginfo("(CAMERA POLE) No aruco detected, retrying...")
        time.sleep(1)
    
    while not rospy.is_shutdown() :
        rst, img = cam.read()

        # If the camera is working
        if rst :  
            if time.time() - last_reset > delay_reset :
                edges = flt.edgeArucoDetection(img, detector)
                if edges is not None :
                    # Try to reset the perspective
                    try :
                        pts1, pts2 = flt.calc_perspective(img, detector)
                        last_reset = time.time()
                        rospy.loginfo("(CAMERA POLE) Perspective reset")
                    except Exception as e :
                        rospy.logerr(f"(CAMERA POLE) Error while resetting the perspective : {e}")

            result_blue, result_yellow = calc_real_pos_and_rot(img, pole_decal, pole_height, ar_height_blue, ar_height_yellow, id_range_blue, id_range_yellow, detector, display)
            print(result_blue, result_yellow)
            if result_blue is not None and result_yellow is not None :
                # Convert the position to meters
                if result_blue != (-1, -1, -1) :
                    result_blue_x = result_blue[0] / 100
                    result_blue_y = result_blue[1] / 100

                if result_yellow != (-1, -1, -1) :
                    result_yellow_x = result_yellow[0] / 100
                    result_yellow_y = result_yellow[1] / 100

                # Publish the position
                pos_rot_pub.publish(Float32MultiArray(data=[result_blue_x, result_blue_y, result_blue[2],
                                                            result_yellow_x, result_yellow_y, result_yellow[2]]))

                rospy.loginfo(f"(CAMERA POLE) Blue robot : {result_blue_x}, {result_blue_y}, {result_blue[2]}")
                rospy.loginfo(f"(CAMERA POLE) Yellow robot : {result_yellow_x}, {result_yellow_y}, {result_yellow[2]}")
                
        else :
            rospy.logerr("(CAMERA POLE) Error while reading the camera")
        
if __name__ == "__main__" :
    main()