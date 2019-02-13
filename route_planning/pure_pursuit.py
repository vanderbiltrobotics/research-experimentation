# Input: array of turning points, current location
# Output: double angular_velocity

# current.pose
# route nav_msgs/path
# publlish twist msg
# recomputing route
# efficient path

path_topic = ""
current_topic = ""
twist_topic = ""

import numpy as np
import math
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
import nav_msgs


class PurePursuit:

    def __init__(self, path_topic, current_topic, twist_topic):
        self.cur_sub = rospy.Subscriber(current_topic, Pose, self.set_current)
        self.path_sub = rospy.Subscriber(path_topic, PoseStamped, self.set_path)
        self.n = len(self.path)
        self.i = [index if (self.path[index] >= self.cur and self.path[index + 1] <= self.cur) else None for index in
                  range(self.n - 1)]
        # 1 random val here
        self.linear_vel = 1
        self.twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=0)
        self.twist = 0
        # self.twist = angular_vel

    # Subscriber Functions

    def set_path(self, new_path):
        self.path = new_path.poses

    def set_current(self, new_current):
        self.cur = new_current.pose

    # Publisher Function

    def send_twist(self):
        self.twist_pub.publish(self.twist)

    # Helper Functions

    # dis: returns distance between two points
    # v1: the first point
    # v2: the second point. defaults to origin
    def dis(self, v1, v2=[0, 0]):
        v1 = np.array(v1)
        v2 = np.array(v2)
        return np.linalg.norm(v1 - v2)

    def genpoint(self, i):
        seglen = self.dis(self.path[i + 1], self.base)
        segv = [self.path[i + 1][0] - self.path[i][0], self.path[i + 1][1] - self.path[i][1]]
        return [self.base[0] + (self.strdist / seglen) * segv[0], self.base[1] + (self.strdist / seglen) * segv[1]]

    # projectbase: returns the point on the path with shortest distance to current location
    # cur: current position coordinates
    # i: the segment of the  path we are in
    def projectbase(self, i):
        if i < self.n:
            if i == self.n - 1:
                return self.path[self.n - 1]
            segv = [self.path[i + 1][0] - self.path[i][0], self.path[i + 1][1] - self.path[i][1]]
            seglen = self.dis(self.path[i + 1], self.path[i]) * 1.0
            curv = [self.cur[0] - self.path[i][0], self.cur[1] - self.path[i][1]]
            curlen = np.dot(curv, segv) / seglen
            print "curlen: {}\ntp[i]: {}\nseglen: {}\ncurv: {}".format(curlen, self.path[i], seglen, curv)
            if curlen <= seglen:
                return [self.path[i][0] + (curlen / seglen) * segv[0], self.path[i][1] + (curlen / seglen) * segv[1]]
            else:
                return self.projectbase(i + 1)
        return None

    # updatestrdist: returns the new length of the distance between look ahead and current location
    # cur: current location
    # i: the segment of path we are in
    def updatestrdist(self, i):
        return 1 + self.dis(self.cur, self.path[i + 1])  # 1 random number here

    # gentarget: returns the coordinates of the target
    # cur: the current position coordinates
    # i: the segment of path we are in
    def gentarget(self, i):
        self.base = self.projectbase(i)
        if self.base is None:
            return None
        disremain = self.dis(self.base, self.path[i + 1])
        if disremain == 0 and i == self.n - 1:
            return self.path[self.n - 1]
        self.strdist = self.updatestrdist(i)
        print "base: {}\nstrdist: {}\ndisremain: {}\ni: {}".format(self.base, self.strdist, disremain, i)
        return self.genpoint(i) if self.strdist <= disremain else self.genpoint(i + 1)

    # linear_vel: returns linear velocity
    def linear_velocity(self):
        return self.linear_vel

    # angular_vel: returns the angular velocity
    def angular_vel(self, lookahead, cur):
        # replace with formula
        # theta = 2 * math.acos(self.cur.orientation.w) along (x, y, z)
        theta = 135
        # clean up and double check the formula
        return 2 / ((np.square(cur[0] - lookahead[0]) + np.square(cur[1] - lookahead[1])) / (self.linear_velocity() * (
                    math.sin(math.radians(theta)) * (lookahead[1] - cur[1]) + math.cos(math.radians(theta)) * (
                        lookahead[0] - cur[0]))))

    # main function
    def calc(self):
        print self.path
        lookahead = self.gentarget(self.i)
        self.twist = self.angular_vel(lookahead, self.cur)
        print "lookahead: {}\n".format(lookahead)
        print "linear velocity: {}\nangular velocity: {}".format(self.linear_velocity(), self.twist)
        self.send_twist()


pp = PurePursuit(path_topic, current_topic, twist_topic)
pp.calc()
