"""Terrain preview from the camera POV
 _____________________
|                     |
|     13       12     |
|                     |
|     11       10     |
|_____________________|

"""




from utils import tagDetector as tagd
import matplotlib.image as mpimg
import cv2
from pathlib import Path
import numpy as np
import yaml

def edgeArucoDetection(img : str) :
    """Detects the aruco at the 4 edges of the image (id 10-13). Returns their position and the size difference between the front and back aruco"""
    #Load image
    img = mpimg.imread(img)

    #Detect arucos
    corners, ids, _ = tagd.recognize_arucos(img, True)
    arPos : list[list[int]] = [[0, 0], [0, 0], [0, 0], [0, 0]]
    nb_ar = 0

    #Process the pos of the arucos
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

def calc_perspective(img : str) :
    """Calculates the perspective transformation from the arucos detected in the image"""

    #Detect arucos, load image
    arpos = edgeArucoDetection(img)
    img = mpimg.imread(img)

    img_height = len(img)
    img_width = int(img_height * 3/2)
    
    #Offsets to the edge of the board
    x_offset = 73 * img_width / 300
    y_offset = 50 * img_height / 200

    #Load config file
    config = yaml.safe_load(open(str(Path(__file__).parent.parent / "config" / "board.yaml")))

    #Calculate the perspective transformation
    pts1 = np.float32([arpos[2], arpos[3], arpos[0], arpos[1]])
    pts2 = np.float32([[img_width - x_offset, y_offset], [x_offset, y_offset], [img_width - x_offset, img_height - y_offset], [x_offset, img_height - y_offset]])
    config["img"]["perspective"]["pts1"] = pts1.tolist()
    config["img"]["perspective"]["pts2"] = pts2.tolist()

    #Write to config file
    with open(str(Path(__file__).parent.parent / "config" / "board.yaml"), "w") as f :
        yaml.dump(config, f)

    return pts1, pts2

def unwarp_img(img : str) :
    """Unwarps the image using the perspective transformation"""

    #load image
    img_arr = mpimg.imread(img)
    img_height = len(img_arr)
    img_width = int(img_height * 3/2)

    #Load config file
    config = yaml.safe_load(open(str(Path(__file__).parent.parent / "config" / "board.yaml")))

    #Get perspective transformation
    pts1 = np.float32(config["img"]["perspective"]["pts1"])
    pts2 = np.float32(config["img"]["perspective"]["pts2"])

    if np.array_equal(pts1, np.zeros((4, 2))) :
        pts1, pts2 = calc_perspective(img)

    #Apply perspective transformation.
    #We are doubling the image's size in case the arucos are outside the board, instead of cropping only the board
        
    #Translation to the center of the new image
    tx = (img_width *  2 - img_width) /  2
    ty = (img_height *  2 - img_height) /  2
    translation = np.float32([[1, 0, tx], [0, 1, ty], [0, 0, 1]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    M = np.dot(translation, M)

    #Apply transformation and save the image
    dst = cv2.warpPerspective(img_arr, M, (img_width*2, img_height*2))
    dst = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)
    cv2.imwrite(str(Path(__file__).parent.parent / "img" / "unwarped_img.jpg"), dst)
    
    return pts1, pts2