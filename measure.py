import yaml
from pathlib import Path
import time
import cProfile
import cv2
import pstats

from utils import flattenimg as flt, tagDetector as tagd, reset, make_detector
from robotpos_calc import calc_real_pos_and_rot


def main(robot_color = 0) :
    """The main function of the module, calculates the real position of the robot from the flattened image of the board.
    Do not forget to modify the config file for the robot's aruco id and the pole's position
    Args:
        robot_color (int, optional): The color of the robot. Defaults to 0. 0 for blue, 1 for yellow.
    """
    reset.reset_perspective()
    
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
    
    cam = cv2.VideoCapture(cam_id)
    x, y, alpha = (0, 0, 0)
    last_reset = time.time()
    
    img = cv2.imread("img/2024-04-17-201248.jpg")
    # If the camera is working
    if time.time() - last_reset > delay_reset :
        edges = flt.edgeArucoDetection(img, detector)
        if edges is not None :
            result = calc_real_pos_and_rot(img, id_range, pole_decal, pole_height, ar_height, detector, True, False)
            last_reset = time.time()
        else : # failed to detect edges
            result = calc_real_pos_and_rot(img, id_range, pole_decal, pole_height, ar_height, detector, False, False)
    else :
        result = calc_real_pos_and_rot(img, id_range, pole_decal, pole_height, ar_height, detector, False, False)
        
    if result is not None :
        x, y, alpha = result


def test_main():
    pr = cProfile.Profile()
    pr.enable()
    for _ in range(10):
        main()
    pr.disable()
    ps = pstats.Stats(pr).sort_stats('cumulative')
    ps.print_stats()

if __name__ == "__main__":
    test_main()