import cv2
import time
from IPython.display import clear_output
import keyboard
from pathlib import Path
import numpy as np

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
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        params = cv2.aruco.DetectorParameters()
        params.detectInvertedMarker = True
        detector = cv2.aruco.ArucoDetector(arucoDict, params)
        
    (corners, ids, rejected) = detector.detectMarkers(img)
    cv2.imwrite(str(default_imgpath / "detected_arucos.jpg"), img)

    if wimg : #if display img
        try :
            img_copy = img.copy()
            for i in range(len(ids)) :
                x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4
                y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4
                print("ID : ", ids[i], "Position: ", x, y)
                cv2.drawMarker(img_copy, (int(x), int(y)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2) #affiche une croix sur le centre de l'aruco
            for i in range(len(rejected)) :
                x = (rejected[i][0][0][0] + rejected[i][0][1][0] + rejected[i][0][2][0] + rejected[i][0][3][0]) / 4
                y = (rejected[i][0][0][1] + rejected[i][0][1][1] + rejected[i][0][2][1] + rejected[i][0][3][1]) / 4
                cv2.drawMarker(img_copy, (int(x), int(y)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.imwrite(str(default_imgpath / "detected_arucos.jpg"), img_copy)
        except Exception as e :
            print("Error while displaying the image : ", e)

    return corners, ids, rejected

def get_rotation(img, id : int, detector : cv2.aruco.ArucoDetector = None) :
    """Returns the rotation of the aruco with the given id. This assumes that the image was flattened.
    
    Args:
        img (np.array): Image to process
        id (int): ID of the aruco to process
        
    Returns:
        rotation (float): the rotation of the aruco, in radians
    """
    
    (corners, ids, _) = detector.detectMarkers(img)
    
    if ids is None :
        print("No arucos detected")
        return 0.
    
    for i in range(len(ids)) :
        if ids[i] == id :
            x1 = corners[i][0][0][0]
            y1 = corners[i][0][0][1]
            x2 = corners[i][0][1][0]
            y2 = corners[i][0][1][1]
            
            center = np.mean(corners[i][0], axis = 0) #center of the aruco
            vectors = np.array([[x1, y1], [x2, y2]]) - center #vectors from the center to the corners
            angles = np.arctan2(vectors[:, 1], vectors[:, 0]) #angles of the vectors
            rotation = float(np.mean(angles)) #mean of the angles
            
            return rotation
    print("Aruco with id {} not found".format(id))
    return 0.
            

# DISPLAY

def display_arucos(img) :
    clear_output(wait=True)
    recognize_arucos(img, True)
    cv2.imshow(str(default_imgpath / "webcam"), img)    
    c = cv2.waitKey(1)

def realtime_detection(cam_id : int) :
    cam = cv2.VideoCapture(cam_id)
    img = None #le C m'a matrixé 
    rst = False
    while True :
        if keyboard.is_pressed('q') :
            break
        rst, img = cam.read()
        if rst :
            cv2.imwrite(str(default_imgpath / "webcam.jpg"), img)
            display_arucos(img)
            time.sleep(0.1)
    
    cam.release()

def add_point_on_img(img, x : int, y : int, output : str) :
    cv2.drawMarker(img, (int(x), int(y)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
    #rgb
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(str(default_imgpath / output), img)
    

def show_real_pos_img(img : np.ndarray, x : float, y : float, alpha : float = 0.) :
    """Displays the image with the calculated position of the robot
    params :
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
    cv2.imwrite(str(default_imgpath / "real_position.jpg"), img_copy)

    cv2.imshow("Real position", img_copy)
    cv2.waitKey(10)