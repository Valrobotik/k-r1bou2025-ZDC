# coding : utf-8
# Nom du fichier : tagDetector.py
# Auteur : zatomos
# Date de création : 21-09-2023
# Modifié par : Paul
# Date de modification : 24-09-2023
# Description : ce script permet de détecter les tag aruco et de les afficher sur l'écran


import cv2
import time
from IPython.display import clear_output
import keyboard
from pathlib import Path

imgpath = Path(__file__).parent.parent / "img"

def recognize_arucos(img, wimg = False) :
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.detectInvertedMarker = True

    detector = cv2.aruco.ArucoDetector(arucoDict, params)
    (corners, ids, rejected) = detector.detectMarkers(img)

    if wimg :
        try :
            for i in range(len(ids)) :
                x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4
                y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4
                print("ID : ", ids[i], "Position: ", x, y)
                cv2.drawMarker(img, (int(x), int(y)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2) #affiche une croix sur le centre de l'aruco
            for i in range(len(rejected)) :
                x = (rejected[i][0][0][0] + rejected[i][0][1][0] + rejected[i][0][2][0] + rejected[i][0][3][0]) / 4
                y = (rejected[i][0][0][1] + rejected[i][0][1][1] + rejected[i][0][2][1] + rejected[i][0][3][1]) / 4
                cv2.drawMarker(img, (int(x), int(y)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.imwrite(str(imgpath / "detected_arucos.jpg"), img)
        except :
            pass #no arucos detected


    return corners, ids, rejected

def display_arucos(img) :
    clear_output(wait=True)
    recognize_arucos(img, True)
    cv2.imshow(str(imgpath / "webcam"), img)    
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
            cv2.imwrite(str(imgpath / "webcam.jpg"), img)
            display_arucos(img)
            time.sleep(0.1)
    
    cam.release()

def add_point_on_img(img, x : int, y : int, output : str) :
    cv2.drawMarker(img, (int(x), int(y)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
    #rgb
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(str(imgpath / output), img)

if __name__ == "__main__" :
    realtime_detection(0)