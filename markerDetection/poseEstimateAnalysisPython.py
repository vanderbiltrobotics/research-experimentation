import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors
from matplotlib.ticker import PercentFormatter

# Takes a formatted .txt file with tvec & rvec pose estimation data and compares them
# Goal: Determine the accuracy and range of simple Aruco Board detection

# list = [[1],2,3]
# print(list[0][0])


''' Experiment constants '''
x = 0.1524  # distance from center to measured corner; meters
theta = [0,5,15,25,35,45]
y0 = [2,4,6]

poseData = open("right_camera_detection/vectors_right_2m_0 d.txt","r")

vecTotal = []
xvec = []
yvec = []
zvec = []
r1vec = []
r2vec = []
r3vec = []
for line in poseData:
    # print(line)

    # split string
    splitLine = line.split(",")
    # print(splitLine)

    # remove \n from last number
    splitLine[len(splitLine)-1] = splitLine[len(splitLine)-1][:-2]
    # print(splitLine)

    # check for incomplete data
    empty = False
    for x in splitLine:
        if x == '':
            empty = True

    if not empty:
        # convert to num
        vec = [float(i) for i in splitLine]
        # print(vec)

        # add to list of all data
        xvec.append(vec[0])
        yvec.append(vec[1])
        zvec.append(vec[2])
        r1vec.append(vec[3])
        r2vec.append(vec[4])
        r3vec.append(vec[5])

print(xvec)
n_bins = 200
fig, axs = plt.subplots(2, 3, tight_layout=True)
axs[0][0].hist(xvec, bins=n_bins)
plt.xlabel("Distance (m)")
axs[0][1].hist(yvec, bins=n_bins)
axs[0][2].hist(zvec, bins=n_bins)
plt.xlabel("Distance (m)")
axs[1][0].hist(r1vec, bins=n_bins)
plt.xlabel("Angle (degrees)")
axs[1][1].hist(r2vec, bins=n_bins)
plt.xlabel("Angle (degrees)")
axs[1][2].hist(r3vec, bins=n_bins)
plt.xlabel("Angle (degrees)")


# axs[1].hist(yvec, bins=n_bins)
plt.show()

# Calculate mean
mean_xvec = np.mean(xvec)
mean_yvec = np.mean(yvec)
mean_zvec = np.mean(zvec)
mean_r1vec = np.mean(r1vec)
mean_r2vec = np.mean(r2vec)
mean_r3vec = np.mean(r3vec)

# Calculate stdev
std_xvec = np.std(xvec)
std_yvec = np.std(yvec)
std_zvec = np.std(zvec)
std_r1vec = np.std(r1vec)
std_r2vec = np.std(r2vec)
std_r3vec = np.std(r3vec)






