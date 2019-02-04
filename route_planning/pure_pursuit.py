# Input: array of turning points, current location
# Output: double linear_velocity, double angular_velocity

import numpy as np
import math
# tp = my_grid.test_search_algo(A_star) #list of point coordinates
tp = []
for i in range(6):
    tp.append([i+1,i])
n = len(tp) #number of turning points

# Helper Functions

# dis: returns distance between two points
# v1: the first point
# v2: the second point. defaults to origin
def dis(v1, v2 = [0,0]):
    v1 = np.array(v1)
    v2 = np.array(v2)
    return np.linalg.norm(v1-v2)

# dotprod: returns the dot product of two vectors
# v1: the first vector
# v2: the second vector. defaults to (1,0)
def dotprod(v1, v2 = [1,0]):
    v1 = np.array(v1)
    v2 = np.array(v2)
    return np.dot(v1,v2)

def trgtmkr(base, strdist, i):
    # return base[0] + cur[0] / dis(cur) * strdist, base[1] + cur[1] / dis(cur) * strdist
    seglen = dis(tp[i+1],base)
    segv = [tp[i+1][0]-tp[i][0], tp[i+1][1]-tp[i][1]]
    return [base[0] + (strdist / seglen) * segv[0], base[1] + (strdist / seglen) * segv[1]]
    # dx = tp[i+1][0] - tp[i][0]
    # dy = tp[i+1][1] - tp[i][1]
    # if dx == 0:
    #     return base[0], base[1] + strdist
    # if dy == 0:
    #     return base[0] + strdist, base[1]
    # return [base[0] + strdist/np.sqrt(1+np.square(dy/dx)), base[1] + (strdist*np.sqrt(1+np.square(dy/dx)))/(dy/dx)]

# projectbase: returns the point on the path with shortest distance to current location
# cur: current position coordinates
# i: the segment of the  path we are in
def projectbase(cur, i):
    if i < n:
        if i == n - 1:
            return tp[n-1]
        # else:
        #     segv = [tp[i+1][0]-tp[i][0], tp[i+1][1]-tp[i][1]]
        #     seglen = dis(tp[i], tp[i+1])
        #     curv = [cur[0]-tp[i][0], cur[1]-tp[i][1]]
        #     curlen = dotprod(curv, segv) / seglen
        # if curlen <= seglen:
        #     #** is exponent
        #     return (curlen/seglen) ** segv + tp[i]
        # else:
        #     projectbase(cur, i+1)
        segv = [tp[i+1][0]-tp[i][0], tp[i+1][1]-tp[i][1]]
        seglen = dis(tp[i+1], tp[i]) * 1.0
        curv = [cur[0] - tp[i][0], cur[1] - tp[i][1]]
        curlen = dotprod(curv, segv) / seglen
        print "curlen: {}\ntp[i]: {}\nseglen: {}\ncurv: {}".format(curlen, tp[i], seglen, curv)
        if curlen <= seglen:
            # return trgtmkr(tp[i], curlen, i)
            return [tp[i][0] + (curlen/seglen)*segv[0], tp[i][1] + (curlen/seglen)*segv[1]]
        else:
            return projectbase(cur, i+1)
    return None

# updatestrdist: returns the new length of the distance between look ahead and current location
# cur: current location
# i: the segment of path we are in
def updatestrdist(cur, i):
    return 1 + dis(cur, tp[i+1]) # 1 random number here

# gentarget: returns the coordinates of the target
# cur: the current position coordinates
# i: the segment of path we are in
def gentarget(cur, i):
    base = projectbase(cur, i)
    if base is None:
        return None
    disremain = dis(base, tp[i+1])
    if disremain == 0 and i == n-1:
        return tp[n-1]
    strdist = 1
    print "base: {}\nstrdist: {}\ndisremain: {}\ni: {}".format(base, strdist, disremain, i)
    return trgtmkr(base, strdist, i) if strdist <= disremain else trgtmkr(tp[i+1], strdist-disremain, i+1)

# linear_vel: returns linear velocity
def linear_vel():
    return 1

# angular_vel: returns the angular velocity
def angular_vel(lookahead, cur):
    theta = 135
    return ((np.square(cur[0]-lookahead[0]) + np.square(cur[1] - lookahead[1])) / (linear_vel()*(math.sin(math.radians(theta))*(lookahead[1]-cur[1]) + math.cos(math.radians(theta))*(lookahead[0]-cur[0]))))

# main function
def pure_pursuit():
    i = 0
    print tp
    cur = [0.5,0.5]     # to be supplied
    lookahead = gentarget(cur, i)
    print "lookahead: {}\n".format(lookahead)
    print "linear velocity: {}\nangular velocity: {}".format(linear_vel(), angular_vel(lookahead, cur))

pure_pursuit()
