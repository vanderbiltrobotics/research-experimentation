# Input: array of turning points, current location
# Output: double linear_velocity, double angular_velocity

import numpy as np
tp = np.array([[1,1],[25,25],[50,50]]) #list of point coordinates
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

# projectbase: returns the point on the path with shortest distance to current location
# cur: current position coordinates
# i: the segment of the  path we are in
def projectbase(cur, i):
    if i < n:
        if i == n - 1:
            return tp[n]
        else:
            segv = [tp[i+1][0]-tp[i][0], tp[i+1][1]-tp[i][1]]
            seglen = dis(tp[i], tp[i+1])
            curv = [cur[0]-tp[i][0], cur[1]-tp[i][1]]
            curlen = dotprod(curv, segv) / seglen
        if curlen <= seglen:
            #** is exponent
            return (curlen/seglen) ** segv + tp[i]
        else:
            projectbase(cur, i+1)
    return None

# updatestrdist: returns the new length of the distance between look ahead and current location
# cur: current location
# i: the segment of path we are in
def updatestrdist(cur, i):
    return 1 + dis(cur, tp[i+1]) # 1 random number here

def trgtmkr(base, cur, strdist):
    return base[0] + cur[0] / dis(cur) * strdist, base[1] + cur[1] / dis(cur) * strdist

# gentarget: returns the coordinates of the target
# cur: the current position coordinates
# i: the segment of path we are in
def gentarget(cur, i):
    base = projectbase(cur, i)
    if base is None:
        return None
    disremain = dis(base, tp[i+1])
    if disremain == 0 and i == n-1:
        return tp[i]
    strdist = disremain if i >= n-1 else updatestrdist(base, i)
    return trgtmkr(base, cur, strdist) if strdist <= disremain else trgtmkr(tp[i+1], cur, strdist-disremain)

# linear_vel: returns linear velocity
def linear_vel(lookahead, cur):
    return 1 + dis(cur, lookahead)
    #TODO
    #    convert length to velocity

# angular_vel: returns the angular velocity
def angular_vel(lookahead, cur):
    curvature = 2*lookahead[0] / dis(lookahead,cur)**2
    #TODO
    #    convert arc length to angular velocity
    return curvature

# main function
def pure_pursuit():
    i = 0
    cur = tp[0]     # to be supplied
    # while i < n:
    lookahead = gentarget(cur, i)
    print "linear velocity: {}\nangular velocity: {}".format(linear_vel(lookahead, cur), angular_vel(lookahead, cur))
    i += 1
