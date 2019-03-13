import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import cv2.aruco as aruco
import glob
import yaml

''' Load camera matrix and distortion coefficients '''
with open('calibration/calibration10.yaml') as f:
    loadeddict = yaml.load(f)

cameraMatrix = np.asarray(loadeddict.get('camera_matrix'))
distCoeff = np.asarray(loadeddict.get('dist_coeff'))

'''Aruco dictionary generation'''
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
# aruco_dict_custom = aruco.custom_dictionary(25,5) # optimize dictionary for inter-marker distance
param = aruco.DetectorParameters_create() # default parameters

'''Aruco marker generation'''
# markerImg = aruco.drawMarker(aruco_dict, 22, 700)
# cv.imshow("Aruco Board", markerImg)
# cv.imwrite('arucoMarker.png',markerImg)
# cv.waitKey(0)
# cv.destroyAllWindows()

'''Aruco board generation'''
markerLength = .029  # m; determine later
markerSeparation = .006  # m; determine later
board = aruco.GridBoard_create(5, 7, markerLength, markerSeparation, aruco_dict)

'''marker detection and pose estimation'''
cap = cv.VideoCapture(0)

'''create file to store data'''
PATH = "" #"data/pose_estimates/mono/"
filename = "test.txt"
f = open(PATH + filename, "a")
count = 0

while 1:
    # Capture frame
    ret, img = cap.read()

    # Convert to HSV
    # img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # detect markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=param)
    aruco.refineDetectedMarkers(img, board, corners, ids, rejectedImgPoints, cameraMatrix, distCoeff)
    # print(corners)

    # estimate pose from Aruco board
    if ids is not None:
        img = aruco.drawDetectedMarkers(img, corners)
        retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeff)
        if retval != 0:
            img_w_axis = aruco.drawAxis(img, cameraMatrix, distCoeff, rvec, tvec, .01)
            print(tvec[2][0])
            if count <= 200:
                f.write(str(tvec[0][0]) + "," + str(tvec[1][0]) + "," + str(tvec[2][0]) + ",")
                f.write(str(rvec[0][0]) + "," + str(rvec[1][0]) + "," + str(rvec[2][0]) + "\n")
                count = count + 1


    # Display the frame
    cv.imshow('Marker Detection', img)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv.destroyAllWindows()