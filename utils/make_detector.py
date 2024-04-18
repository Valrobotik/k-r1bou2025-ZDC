import cv2

def make_detector() :
    # Dictionary of the 100 first IDs
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    params = cv2.aruco.DetectorParameters()
    # Make detector more lenient
    params.detectInvertedMarker = True
    detector = cv2.aruco.ArucoDetector(arucoDict, params)
    
    return detector