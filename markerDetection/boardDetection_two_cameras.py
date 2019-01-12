import cv2 as cv
import numpy as np
import cv2.aruco as aruco
import yaml

''' Load camera matrix and distortion coefficients '''
with open('calibration10.yaml') as f:
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
cap2 = cv.VideoCapture(1)

'''create file to store data'''
f = open("vectors.txt", "a")
count = 0

while 1:
    # Capture frame
    ret, img = cap.read()
    ret2, img2 = cap2.read()

    # Convert to HSV
    # img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # detect markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=param)
    corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(img2, aruco_dict, parameters=param)
    aruco.refineDetectedMarkers(img, board, corners, ids, rejectedImgPoints, cameraMatrix, distCoeff)
    aruco.refineDetectedMarkers(img2, board, corners2, ids2, rejectedImgPoints2, cameraMatrix, distCoeff)

    # estimate pose from Aruco board
    if (ids is not None) or (ids2 is not None):
        cv.imwrite('left_aruco.png', img)
        cv.imwrite('right_aruco.png', img2)
        img = aruco.drawDetectedMarkers(img, corners)
        img2 = aruco.drawDetectedMarkers(img2, corners2)
        retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeff)
        retval2, rvec2, tvec2 = aruco.estimatePoseBoard(corners2, ids2, board, cameraMatrix, distCoeff)
        if retval != 0 and retval2 != 0:
            img_w_axis = aruco.drawAxis(img, cameraMatrix, distCoeff, rvec, tvec, .01)
            img_w_axis2 = aruco.drawAxis(img2, cameraMatrix, distCoeff, rvec2, tvec2, 0.01)
            print(tvec[2][0])
            if count <= 200:
                f.write(str(tvec[0][0]) + "," + str(tvec[1][0]) + "," + str(tvec[2][0]) + ",")
                f.write(str(rvec[0][0]) + "," + str(rvec[1][0]) + "," + str(rvec[2][0]) + "\n")
                count = count + 1


    # Display the frame
    cv.imshow('Left Camera Marker Detection', img)
    cv.imshow('Right Camera Marker Detection', img2)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv.destroyAllWindows()