import cv2.aruco as aruco
import cv2

marker_length = 0.04
marker_separation = .01

dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
board = aruco.GridBoard_create(5, 7, marker_length, marker_separation, dict)

retval = cv2.aruco_GridBoard.getGridSize(board)

img = board.draw(outSize=(5000, 8000), marginSize=10, borderBits=1)
cv2.imwrite("test_gridboard.jpg", img)
cv2.imshow('Gridboard', img)

cv2.waitKey(0)
cv2.destroyAllWindows()

