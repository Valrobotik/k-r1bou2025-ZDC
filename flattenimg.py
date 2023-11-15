"""Terrain preview from the camera POV
 _____________________
|                     |
|     13       12     |
|                     |
|     11       10     |
|_____________________|

"""




import utils.tagDetector as tagd
import matplotlib.image as mpimg
import cv2
from pathlib import Path
import numpy as np

def edgeArucoDetection(img : str) :
    """Detects the aruco at the 4 edges of the image (id 10-13). Returns their position and the size difference between the front and back aruco"""
    img = mpimg.imread(img)

    corners, ids, _ = tagd.recognize_arucos(img, True)
    arPos : list[list[int]] = [[0, 0], [0, 0], [0, 0], [0, 0]]
    nb_ar = 0

    for i in range(len(ids)) :
        if ids[i][0] < 20 or ids[i][0] > 23 :
            continue
        nb_ar += 1
        x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4
        y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4

        arPos[ids[i][0] - 20] = [x, y]
    
    if nb_ar != 4 :
        raise Exception("Not enough arucos detected")
    return arPos

def unwarp_img(img : str) :
    arpos = edgeArucoDetection(img)
    img = mpimg.imread(img)
    img_height = len(img)
    img_width = int(img_height * 3/2)
    x_offset = 73 * img_width / 300
    y_offset = 50 * img_height / 200

    pts1 = np.float32([arpos[2], arpos[3], arpos[0], arpos[1]])
    pts2 = np.float32([[img_width - x_offset, y_offset], [x_offset, y_offset], [img_width - x_offset, img_height - y_offset], [x_offset, img_height - y_offset]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    dst = cv2.warpPerspective(img, M, (img_width, img_height))
    dst = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)
    cv2.imwrite(str(Path(__file__).parent / "img" / "unwarped_img.jpg"), dst)
    return dst

def calc_real_pos(flat_img : str, pole_decal : int, height_pole : int, ar_id : int) :
    """Calculates the real position of the robot from the flattened image, according to the position of the pole
    params :
        flat_img : the flattened image
        pole_decal : the distance between the pole and the center of the board : negative value -> left, positive value -> right
        height_pole : the height of the pole
        ar_id : the id of the aruco we want to detect
    """

    flat_img = mpimg.imread(flat_img)
    img_height = len(flat_img)
    img_width = len(flat_img[0])
    ar_height = 53 #height of the robot's aruco from the ground
    corners, ids, _ = tagd.recognize_arucos(flat_img, True)
    
    for i in range(len(ids)) :
        if ids[i][0] == ar_id :
            x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4
            y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4
            
            #conversion from pixels to meters
            x = 300 * x / img_width
            y = 200 * y / img_height

            #calculate the distance from the aruco
            pos_camera = 150 + pole_decal
            dist_x = abs(x - pos_camera)
            print(dist_x)

            #invert y axis
            y = 200 - y

            alphax = np.arctan(dist_x / height_pole)
            alphay = np.arctan(y / height_pole)
            
            lx = (height_pole - ar_height) * np.tan(alphax)
            ly = (height_pole - ar_height) * np.tan(alphay)

            posx = 150+lx if x > pos_camera else 150-lx
            print(posx, ly)
            return posx, ly
            
    print("Aruco with id {} not found".format(ar_id))
    return None

def show_real_pos_img(img : str, x : float, y : float) :
    img = mpimg.imread(img)
    img_height = len(img)
    img_width = len(img[0])
    x = x * img_width / 300
    y = 200 - y
    y = y * img_height / 200
    tagd.add_point_on_img(img, x, y, "real_pos.jpg")

unwarp_img(str(Path(__file__).parent / "img" / "imgboard8.jpg"))
x, y = calc_real_pos(str(Path(__file__).parent / "img" / "unwarped_img.jpg"), 0, 100, 5)
show_real_pos_img(str(Path(__file__).parent / "img" / "unwarped_img.jpg"), x, y)