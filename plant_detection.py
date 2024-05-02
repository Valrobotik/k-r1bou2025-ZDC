import cv2
import numpy as np

LOWER_GREEN = (0, 100, 0)
UPPER_GREEN = (100, 255, 100)
ZONES_TO_AVOID = [(100, 0, 200, 20),
                  (52, 0, 100, 15),
                  (200, 0, 248, 15),
                  (0, 116, 12, 165),
                  (0, 40, 12, 88),
                  (288, 116, 300, 165),
                  (288, 40, 300, 88),
                  (138, 80, 162, 98)
                  ]

def convert_coords(x, y, img_wigth, img_height):
    """Converts the image coordinates to the real world coordinates"""
    x = x * 300 / img_wigth
    y = 220 - (y * 220 / img_height)

    return x, y

def is_in_zone(x, y):
    """
    Since some zones are green, we need to avoid them
    Args:
        x (int): x coordinate
        y (int): y coordinate
    Returns:
        bool: True if the point is in a zone to avoid, False otherwise
    """

    for zone in ZONES_TO_AVOID:
        if zone[0] <= x <= zone[2] and zone[1] <= y <= zone[3]:
            return True
    return False


def find_plants(flat_img : np.ndarray) :
    """
    Find the plants in the image
    Args:
        flat_img (np.ndarray): the flattened image
    Returns:
        list: a list of the plants' coordinates
    """

    # Crop the image to keep the board only (len/4 to len*3/4)
    flat_img = flat_img[flat_img.shape[0]//5:flat_img.shape[0]*3//4, flat_img.shape[1]//4:flat_img.shape[1]*3//4]
    plant_list = []

    # Detect green color in the image
    mask = cv2.inRange(flat_img, LOWER_GREEN, UPPER_GREEN)
    # Apply a Gaussian blur to the mask to reduce noise
    blurred = cv2.GaussianBlur(mask, (5, 5), 0)
    # Apply a threshold to the blurred image
    _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)
    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Draw the contours on the original image
    # Keep only the contours that are large enough
    min_area = 200
    large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
    
    # Draw on the middle of the contour
    for cnt in large_contours:
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cx, cy = convert_coords(cx, cy, flat_img.shape[1], flat_img.shape[0])
            if not is_in_zone(cx, cy):
                plant_list.append((cx, cy))        
    return plant_list

def calc_real_pos(flat_img : np.ndarray, pole_decal : int, pole_height : int, plant_height : int):
    """Calculates the real position of the plants from the flattened image, according to the position of the pole
    params :
        flat_img : the flattened image
        pole_decal : the distance between the pole and the center of the board : negative value -> left, positive value -> right
        pole_height : the height of the pole
        plant_height : the height of the plants
    """
    
    real_pos = []

    plant_list = find_plants(flat_img)
    
    #Process the position of the aruco
    for i in plant_list:
            x, y = i

            #Calculate the distance from the aruco
            pos_camera = 150 + pole_decal
            dist_x = abs(x - pos_camera)

            #Angles
            alphax = np.arctan(dist_x / pole_height)
            alphay = np.arctan(y / pole_height)
            
            #Real position
            lx = (pole_height - plant_height) * np.tan(alphax)
            ly = (pole_height - plant_height) * np.tan(alphay)
            posx = 150+lx if x > pos_camera else 150-lx

            real_pos.append((posx, ly))
            
    return real_pos