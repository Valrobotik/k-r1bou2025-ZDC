"""Determines the position of a robot on the board from a flattened image of the board."""

from pathlib import Path
import numpy as np
import yaml
import cv2
import time
import logging

from utils import flattenimg as flt, tagDetector as tagd, reset, make_detector
import plant_detection

# Logger
logging.basicConfig(level = logging.INFO)
logger = logging.getLogger(__name__)
with open(str(Path(__file__).parent / "config" / "board.yaml"), "r") as f :
    config = yaml.safe_load(f)
    logger.setLevel(config["logger"])
    pts1 = np.float32(config["img"]["perspective"]["pts1"])
    pts2 = np.float32(config["img"]["perspective"]["pts2"])

def get_rotation(img, id_range : list, detector : cv2.aruco.ArucoDetector = None) :
    """Returns the rotation of the aruco with the given id. This assumes that the image was flattened.
    
    Args:
        img (np.array): Image to process
        id (list): range id of the aruco to process
        detector (cv2.aruco.ArucoDetector, optional): The aruco detector. Defaults to None, but should be set for better performance.
        
    Returns:
        rotation (float): the rotation of the aruco, in radians
    """
    if detector is None :
        detector = make_detector.make_detector()
    
    (corners, ids, _) = detector.detectMarkers(img)
    
    if ids is None :
        logger.error("No aruco found while calculating the rotation")
        return 0.
    
    for i in range(len(ids)) :
        if ids[i] in id_range :
            x1 = corners[i][0][3][0]
            y1 = corners[i][0][3][1]
            x2 = corners[i][0][0][0]
            y2 = corners[i][0][0][1]
            
            rotation = np.arctan2((y2 - y1), (x2 - x1))

            return rotation
        
    logger.error("Aruco in range {} not found while calculating the rotation".format(id_range))
    return 0.

def calc_real_pos(flat_img : np.ndarray, pole_decal : int, pole_height : int, ar_height : int, id_range : list, detector : cv2.aruco.ArucoDetector = None) :
    """Calculates the real position of the robot from the flattened image, according to the position of the pole
    params :
        flat_img : the flattened image
        pole_decal : the distance between the pole and the center of the board : negative value -> left, positive value -> right
        pole_height : the height of the pole
        ar_height : the height of the aruco
        id_range : the range of the aruco id to detect
        detector : the aruco detector
    """
    
    #Load the image and the arucos
    img_height = len(flat_img) // 2
    img_width = len(flat_img[0]) // 2
    corners, ids, _ = tagd.recognize_arucos(flat_img, True, detector)
    
    if ids is None :
        logger.error("No aruco found while calculating the position")
        return -1, -1
    
    #Process the position of the aruco
    for i in range(len(ids)) :
        if ids[i][0] in id_range :
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
            lx = (pole_height - ar_height) * np.tan(alphax)
            ly = (pole_height - ar_height) * np.tan(alphay)
            posx = 150+lx if x > pos_camera else 150-lx

            return posx, ly
            
    logger.error("Aruco in range {} not found while calculating the position".format(id_range))
    return -1, -1

def calc_real_pos_and_rot(img : np.ndarray, id_range : list, pole_decal : int, pole_height : int, ar_height : int, detector : cv2.aruco.ArucoDetector = None, reset = False, show = False) :
    """Calculates the real position of the robot from the camera.
    
    Args:
        img (np.ndarray): The image taken by the camera
        id_range (list): The range id of the robot's aruco
        cam (cv2.VideoCapture): The camera
        config (dict): The config dictionary
        detector (cv2.aruco.ArucoDetector, optional): The aruco detector. Defaults to None, but should be set for better performance.
        reset (bool, optional): If True, resets the perspective. Defaults to False.
        
    Returns:
        tuple: the x, y and alpha values of the robot
    """
    global pts1, pts2
    
    unwarped_img = flt.unwarp_img(img, detector, (pts1, pts2))

    plant_list = plant_detection.calc_real_pos(unwarped_img, pole_decal, pole_height, 5, True)
    logger.info("Plants : {}".format(plant_list))
    
    # Get the position of the robot
    x, y = calc_real_pos(unwarped_img, pole_decal, pole_height, ar_height, id_range, detector)
    
    if x == -1 and y == -1 : #failed to detect the aruco
        return -1, -1, -1
    
    # Get the rotation of the robot
    alpha = get_rotation(unwarped_img, id_range, detector)
    
    logger.info("Position : {}, {}, Rotation : {}".format(x, y, alpha * 180 / np.pi))
    
    if show :
        tagd.show_real_pos_img(unwarped_img, x, y, alpha)
    
    if reset :
        try :
            pts1, pts2 = flt.calc_perspective(img, detector)
            logger.info("Perspective reset")
        except Exception as e :
            logger.error("Error while resetting the perspective : ", e)
    return x, y, alpha

def main(robot_color = 0) :
    """The main function of the module, calculates the real position of the robot from the flattened image of the board.
    Do not forget to modify the config file for the robot's aruco id and the pole's position
    Args:
        robot_color (int, optional): The color of the robot. Defaults to 0. 0 for blue, 1 for yellow.
    """
    reset.reset_perspective()
    logger.info("Starting")
    
    # Load config file
    config = yaml.safe_load(open(str(Path(__file__).parent / "config" / "board.yaml")))
    id_range_start = config["robots"]["blue_id_range"][0] if robot_color == 0 else config["robots"]["yellow_id_range"][0]
    id_range_end = config["robots"]["blue_id_range"][1] if robot_color == 0 else config["robots"]["yellow_id_range"][1]
    id_range = range(id_range_start, id_range_end + 1)
    delay_reset = config["delay_reset"]
    ar_height = config["robots"]["blue_ar_height"] if robot_color == 0 else config["robots"]["yellow_ar_height"]
    pole_decal = config["pole"]["decal"]
    pole_height = config["pole"]["height"]
    cam_id = config["cam_id"]
    display = True if config["display"] == 1 else False
    
    detector = make_detector.make_detector()
    time_exec = time.time()
    
    cam = cv2.VideoCapture(cam_id)
    x, y, alpha = (0, 0, 0)
    last_reset = time.time()
    
    while True :
        rst, img = cam.read()

        # If the camera is working
        if rst :  
            if time.time() - last_reset > delay_reset :
                edges = flt.edgeArucoDetection(img, detector)
                if edges is not None :
                    result = calc_real_pos_and_rot(img, id_range, pole_decal, pole_height, ar_height, detector, True, display)
                    last_reset = time.time()
                else : # failed to detect edges
                    result = calc_real_pos_and_rot(img, id_range, pole_decal, pole_height, ar_height, detector, False, display)
            else :
                result = calc_real_pos_and_rot(img, id_range, pole_decal, pole_height, ar_height, detector, False, display)
                
            if result is not None :
                x, y, alpha = result
        else :
            logger.error("Error while reading the camera")
        logger.debug("Execution time : {}".format(time.time() - time_exec))
        time_exec = time.time()
            
        
if __name__ == "__main__" :
    main()