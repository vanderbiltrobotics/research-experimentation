# This is the pseudo code for pure pursuit
# Input: array of turning points
# Output: double linear_velocity, double angular_velocity

from math import sqrt
tp = [] #list of point coordinates
n = len(tp)

# Helper Functions

# dis: returns distance between two points
# v1: the first point
# v2: the second point. defaults to origin
def dis(v1, v2 = (0,0)):
    return sqrt((v1[0]-v2[0])^2 + (v1[1]-v2[1])^2)

# dotprod: returns the dot product of two vectors
# v1: the first vector
# v2: the second vector. defaults to (1,0)
def dotprod(v1, v2 = (1,0)):
    return v1[0]*v2[0] + v1[1]*v2[1]

# projectbase: returns the point on the path with shortest distance to current location
# cur: current position coordinates
# i: the segment of the  path we are in
def projectbase(cur, i):
    if i < n:
        segv = (tp[i+1][0]-tp[i][0], tp[i+1][1]-tp[i][1])
        seglen = dis(tp[i], tp[i+1])
        curlen = dotprod(cur, segv)
        if curlen <= seglen:
            return (curlen/seglen) * segv
        else:
            projectbase(cur, i+1)
    return None

# updatestrdist: returns the new length of the distance between look ahead and current location
def updatestrdist():
    strdist = 20
    return strdist

# gentarget: returns the coordinates of the target
# cur: the current position coordinates
# i: the segment of path we are in
def gentarget(cur, i):
    base = projectbase(cur, i)
    disremain = dis(base, tp[i+1])
    strdist = updatestrdist()
    #TODO: how to check the target will be in the next segment or this one
#   if disremain >= strdist:
#       target = base + cur / cur.length() * strdist
#       return target
#   else:
#       gentarget(strdist - disremain, cur->next)

# linear_vel: returns linear velocity
def linear_vel(*args):
    pass

# angular_vel: returns the angular velocity
def angular_vel(*args):
    pass

#main function
def pure_pursuit():
    pass
