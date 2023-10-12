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

def edgeArucoDetection(img : str) :
    """Detects the aruco at the 4 edges of the image (id 10-13). Returns their position and the size difference between the front and back aruco"""
    img = mpimg.imread(img)

    corners, ids, _ = tagd.recognize_arucos(img, True)
    arPos : list[list[int]] = [[0, 0], [0, 0], [0, 0], [0, 0]]

    for i in range(len(ids)) :
        x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4
        y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4

        arPos[ids[i][0] - 20] = [x, y]

    return arPos, sizeDelta(corners)

def sizeDelta(corners : list[list[list[int]]]) :
    """Returns how much bigger the front aruco is compared to the back one, and how much bigger the left one is compared to the right one"""

    frontright = corners[0][0]
    frontleft = corners[1][0]
    backright = corners[2][0]
    backleft = corners[3][0]


    widthleft = (abs(frontleft[0][0] - frontleft[1][0]) + abs(frontleft[2][0] - frontleft[3][0])) / 2
    widthright = (abs(frontright[0][0] - frontright[1][0]) + abs(frontright[2][0] - frontright[3][0])) / 2
    heightfront = (abs(frontleft[0][1] - frontleft[2][1]) + abs(frontleft[1][1] - frontleft[3][1])) / 2
    heightback = (abs(backleft[0][1] - backleft[2][1]) + abs(backleft[1][1] - backleft[3][1])) / 2

    
    return widthleft / widthright, heightfront / heightback


def findBoardEdges(img : str) :
    """Returns the position of the 4 edges of the board, according to the detected arucos"""
    arPos, (sizeDeltax, sizeDeltay) = edgeArucoDetection(img)
    print(sizeDeltax, sizeDeltay)

    espxfront = arPos[0][0] - arPos[1][0]
    espxback = arPos[2][0] - arPos[3][0]
    espy = ((arPos[2][1] - arPos[0][1]) + (arPos[3][1] - arPos[1][1])) / 2

    edgesPos : list[list[int]] = [[0, 0], [0, 0], [0, 0], [0, 0]]

    edgesPos[0] = [arPos[1][0] - espxfront/2*sizeDeltax, arPos[1][1] - espy/2*sizeDeltay] #bottom left
    edgesPos[1] = [arPos[0][0] + espxfront/2*1/sizeDeltax, arPos[0][1] - espy/2*sizeDeltay] #bottom right
    edgesPos[2] = [arPos[3][0] - espxback/2*sizeDeltax, arPos[3][1] + espy/2*1/sizeDeltay] #top left
    edgesPos[3] = [arPos[2][0] + espxback/2*1/sizeDeltax, arPos[2][1] + espy/2*1/sizeDeltay] #top right

    return edgesPos

def drawOnImg(img : str) :
    edgePos = findBoardEdges(img)
    img = mpimg.imread(img)
    for i in range(len(edgePos)) :
        cv2.drawMarker(img, (int(edgePos[i][0]), int(edgePos[i][1])), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
    cv2.imwrite(str(Path(__file__).parent / "img" / "board_edges.jpg"), img)

if __name__ == "__main__" :
    img = "img\imgboard.jpg"
    print(findBoardEdges(img))
    drawOnImg(img)