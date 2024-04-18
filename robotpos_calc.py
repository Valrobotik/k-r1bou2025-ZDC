"""Determines the position of a robot on the board from a flattened image of the board."""

from utils import flattenimg as flt, tagDetector as tagd, reset
from pathlib import Path
import numpy as np
import yaml
import cv2
import time

def calc_real_pos(flat_img : np.ndarray, pole_decal : int, height_pole : int, ar_id : int, detector : cv2.aruco.ArucoDetector = None) :
    """Calculates the real position of the robot from the flattened image, according to the position of the pole
    params :
        flat_img : the flattened image
        pole_decal : the distance between the pole and the center of the board : negative value -> left, positive value -> right
        height_pole : the height of the pole
        ar_id : the id of the aruco we want to detect
    """
    print("Calculating real position")
    #Load the image and the arucos
    img_height = len(flat_img) // 2
    img_width = len(flat_img[0]) // 2
    ar_height =44 #height of the robot's aruco from the ground
    corners, ids, _ = tagd.recognize_arucos(flat_img, True, detector)
    
    if ids is None :
        print("No aruco found")
        return 0, 0
    
    #Process the position of the aruco
    for i in range(len(ids)) :
        if ids[i][0] == ar_id :
            x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4
            y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4
            
            #Conversion from pixels to meters
            x = 300 * (x - img_width / 2) / img_width
            y = 200 * (y - img_height / 2) / img_height
            print(x,y)

            #Calculate the distance from the aruco
            pos_camera = 150 + pole_decal
            dist_x = abs(x - pos_camera)

            #Invert y axis
            y = 200 - y

            #Angles
            alphax = np.arctan(dist_x / height_pole)
            alphay = np.arctan(y / height_pole)
            
            #Real position
            lx = (height_pole - ar_height) * np.tan(alphax)
            ly = (height_pole - ar_height) * np.tan(alphay)
            posx = 150+lx if x > pos_camera else 150-lx

            print("Real position : ", posx, ly)
            return posx, ly
            
    print("Aruco with id {} not found".format(ar_id))
    return None

def calc_real_pos_and_rot(img : np.ndarray, id : int, config : dict, detector : cv2.aruco.ArucoDetector = None, reset = False) :
    """Calculates the real position of the robot from the camera.
    
    Args:
        id (int): The id of the robot's aruco
        cam (cv2.VideoCapture): The camera
        config (dict): The config dictionary
        detector (cv2.aruco.ArucoDetector, optional): The aruco detector. Defaults to None, but should be set for better performance.
        
    Returns:
        tuple: the x, y and alpha values of the robot
    """
        
    unwarped_img = flt.unwarp_img(img, detector)
    x, y = calc_real_pos(unwarped_img, config["pole"]["decal"], config["pole"]["height"], id, detector)
    alpha = tagd.get_rotation(unwarped_img, id, detector)
    tagd.show_real_pos_img(unwarped_img, x, y, alpha)
    
    if reset :
        try :
            flt.calc_perspective(img, detector)
            print("Perspective reset")
        except Exception as e :
            print("Error while calculating the perspective : ", e)
    return x, y, alpha
    # except Exception as e :
    #     print("Error while calculating the real position : ", e)
    #     return None

def main(robot_friendly = True, cam_id = 0) :
    """The main function of the module, calculates the real position of the robot from the flattened image of the board.
    Do not forget to modify the config file for the robot's aruco id and the pole's position
    params :
        robot_friendly : if True, our robot's aruco is detected. If False, the enemy's aruco is detected.
    """
    reset.reset_perspective()
    
    # Load config file
    config = yaml.safe_load(open(str(Path(__file__).parent / "config" / "board.yaml")))
    id = config["robots"]["friendly_id"] if robot_friendly else config["robots"]["enemy_id"]
    delay_reset = config["delay_reset"]
    
    # Dictionary of the 250 first IDs
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    params = cv2.aruco.DetectorParameters()
        #make detector more lenient
    params.detectInvertedMarker = True
    detector = cv2.aruco.ArucoDetector(arucoDict, params)
    
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
                    result = calc_real_pos_and_rot(img, id, config, detector, True)
                    last_reset = time.time()
                else : # failed to detect edges
                    result = calc_real_pos_and_rot(img, id, config, detector, False)
            else :
                result = calc_real_pos_and_rot(img, id, config, detector, False)
                
            if result is not None :
                x, y, alpha = result
                print("Position : ", x, y, alpha * 180 / np.pi)
        else :
            print("Error while reading the camera")
            
        
if __name__ == "__main__" :
    main()