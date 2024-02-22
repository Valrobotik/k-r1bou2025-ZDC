"""Determines the position of a robot on the board from a flattened image of the board."""

from utils import flattenimg as flt, tagDetector as tagd
import matplotlib.image as mpimg
from pathlib import Path
import numpy as np
import yaml

def calc_real_pos(flat_img : str, pole_decal : int, height_pole : int, ar_id : int) :
    """Calculates the real position of the robot from the flattened image, according to the position of the pole
    params :
        flat_img : the flattened image
        pole_decal : the distance between the pole and the center of the board : negative value -> left, positive value -> right
        height_pole : the height of the pole
        ar_id : the id of the aruco we want to detect
    """
    #Load the image and the arucos
    flat_img = mpimg.imread(flat_img)
    img_height = len(flat_img) // 2
    img_width = len(flat_img[0]) // 2
    ar_height = 51 #height of the robot's aruco from the ground
    corners, ids, _ = tagd.recognize_arucos(flat_img, True)
    
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

def show_real_pos_img(img : str, x : float, y : float) :
    img = mpimg.imread(img)
    img_height = len(img) // 2
    img_width = len(img[0]) // 2
    x = x * img_width / 300 + img_width / 2
    y = 200 - y
    y = y * img_height / 200 + img_height / 2
    tagd.add_point_on_img(img, x, y, "real_pos.jpg")

def main(robot_friendly = True) :
    """The main function of the module. It calculates the position of the robot and displays it on the image:
    Do not forget to modify the config file for the robot's aruco id and the pole's position
    params :
        robot_friendly : if True, our robot's aruco is detected. If False, the enemy's aruco is detected.
    """
    config = yaml.safe_load(open(str(Path(__file__).parent / "config" / "board.yaml")))
    unwarped_img = str(Path(__file__).parent / "img" / "unwarped_img.jpg")

    flt.unwarp_img(str(Path(__file__).parent / "img" / config["img"]["name"]))
    id = config["robots"]["friendly_id"] if robot_friendly else config["robots"]["enemy_id"]
    try :
        x, y = calc_real_pos(unwarped_img, config["pole"]["decal"], config["pole"]["height"], id)
        show_real_pos_img(unwarped_img, x, y)
        print(x, y)
    except :
        print("Robot's aruco not found")