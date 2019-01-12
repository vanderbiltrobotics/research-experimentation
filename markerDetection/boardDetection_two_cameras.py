import cv2 as cv
import numpy as np
import cv2.aruco as aruco
import yaml
import os

left_cam = 2
right_cam = 1
id_label = '2m_20 d'


''' Load camera matrix and distortion coefficients '''
with open('camera_left.yaml') as f:
    loadeddict = yaml.load(f)

cameraMatrix_left = np.asarray(loadeddict.get('camera_matrix'))
distCoeff_left = np.asarray(loadeddict.get('dist_coefficients'))

with open('camera_right.yaml') as f:
    loadeddict = yaml.load(f)

cameraMatrix_right = np.asarray(loadeddict.get('camera_matrix'))
distCoeff_right = np.asarray(loadeddict.get('dist_coefficients'))

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
cap = cv.VideoCapture(left_cam)
cap2 = cv.VideoCapture(right_cam)

'''create file to store data'''
f = open("./left_camera_detection/vectors_left_" + id_label + ".txt", "a")
f2 = open("./right_camera_detection/vectors_right_" + id_label + ".txt", "a")
count = 0
img_count = 1

while 1:
    # Capture frame
    ret, img = cap.read()
    ret2, img2 = cap2.read()

    # Convert to HSV
    # img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # detect markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=param)
    corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(img2, aruco_dict, parameters=param)
    aruco.refineDetectedMarkers(img, board, corners, ids, rejectedImgPoints, cameraMatrix_left, distCoeff_left)
    aruco.refineDetectedMarkers(img2, board, corners2, ids2, rejectedImgPoints2, cameraMatrix_right, distCoeff_right)

    # estimate pose from Aruco board
    if (ids is not None) and (ids2 is not None):
        if img_count <= 1:
            cv.imwrite('./left_camera_board_pics/left_aruco_' + id_label + '.png', img)
            cv.imwrite('./right_camera_board_pics/right_aruco_' + id_label + '.png', img2)
            img_count += 1
        img = aruco.drawDetectedMarkers(img, corners)
        img2 = aruco.drawDetectedMarkers(img2, corners2)
        retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, cameraMatrix_left, distCoeff_left)
        retval2, rvec2, tvec2 = aruco.estimatePoseBoard(corners2, ids2, board, cameraMatrix_right, distCoeff_right)
        if retval != 0 and retval2 != 0:
            img_w_axis = aruco.drawAxis(img, cameraMatrix_left, distCoeff_left, rvec, tvec, .01)
            img_w_axis2 = aruco.drawAxis(img2, cameraMatrix_right, distCoeff_right, rvec2, tvec2, 0.01)
            # print(tvec[2][0])
            if count <= 500:
                f.write(str(tvec[0][0]) + "," + str(tvec[1][0]) + "," + str(tvec[2][0]) + ",")
                f.write(str(rvec[0][0]) + "," + str(rvec[1][0]) + "," + str(rvec[2][0]) + "\n")

                f2.write(str(tvec2[0][0]) + "," + str(tvec2[1][0]) + "," + str(tvec2[2][0]) + ",")
                f2.write(str(rvec2[0][0]) + "," + str(rvec2[1][0]) + "," + str(rvec2[2][0]) + "\n")
                count = count + 1
            if count > 500:
                print("Done!")

    # Display the frame
    cv.imshow('Left Camera Marker Detection', img)
    cv.imshow('Right Camera Marker Detection', img2)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv.destroyAllWindows()