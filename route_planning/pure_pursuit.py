# Input: array of turning points, current location
# Output: double linear_velocity, double angular_velocity

import numpy as np
tp = np.array([]) #list of point coordinates
n = len(tp) #number of turning points

# Helper Functions

# dis: returns distance between two points
# v1: the first point
# v2: the second point. defaults to origin
def dis(v1, v2 = (0,0)):
    return np.linalg.norm(v1 - v2)

# dotprod: returns the dot product of two vectors
# v1: the first vector
# v2: the second vector. defaults to (1,0)
def dotprod(v1, v2 = (1,0)):
    return np.dot(v1,v2)

# projectbase: returns the point on the path with shortest distance to current location
# cur: current position coordinates
# i: the segment of the  path we are in
def projectbase(cur, i):
    if i < n:
        if ( i == n - 1 ):
            return tp[n]
        else:
            segv = (tp[i+1][0]-tp[i][0], tp[i+1][1]-tp[i][1])
            seglen = dis(tp[i], tp[i+1])
            curv = (cur[0]-tp[i][0], cur[1]-tp[i][1])
            curlen = dotprod(curv, segv) / seglen
        if curlen <= seglen:
            return (curlen/seglen) ** segv + tp[i]
        else:
            projectbase(cur, i+1)
    return None

# updatestrdist: returns the new length of the distance between look ahead and current location
# cur: current location
# i: the segment of path we are in
def updatestrdist(cur, i):
    return 5 + 20 * dis(cur, tp[i+1]) # 20 random number here

def trgtmkr(base, cur, strdist):
    return ((base[0] + cur[0] / dis(cur) * strdist), (base[1] + cur[1] / dis(cur) * strdist))

# gentarget: returns the coordinates of the target
# cur: the current position coordinates
# i: the segment of path we are in
def gentarget(cur, i):
    base = projectbase(cur, i)
    disremain = dis(base, tp[i+1])
    strdist = updatestrdist(base, i)
    target = trgtmkr(base, cur, strdist)
    if strdist <= disremain:
      return target
    else:
      gentarget(cur, i+1)

# linear_vel: returns linear velocity
def linear_vel(*args):
    pass

# angular_vel: returns the angular velocity
def angular_vel(*args):
    pass

#main function
def pure_pursuit():
    pass
