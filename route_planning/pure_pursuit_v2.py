# Input: array of turning points, current location
# Output: double angular_velocity

#current.pose
#route nav_msgs/path
#publlish twist msg
#recomputing route
#efficient path

path_topic = ""
current_topic = ""
twist_topic = ""

import numpy as np
import math
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import nav_msgs

class PurePursuit:

    def __init__(self, path_topic, current_topic, twist_topic):
        self.cur_sub = rospy.Subscriber(current_topic, Pose, self.set_current)
        self.path_sub = rospy.Subscriber(path_topic, PoseStamped, self.set_path)
        self.n = len(self.path)
        # 1 random val here
        self.linear_vel = 1
        self.twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=0)
        self.twist = Twist()
        self.base = self.projectbase()
        self.i = -1
        #self.twist = angular_vel

        # Set initial lookahead distance
        self.lookahead_dist = 1.0

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
    def dis(self, v1, v2 = [0,0]):
        v1 = np.array(v1)
        v2 = np.array(v2)
        return np.linalg.norm(v1-v2)

    # genpoint: returns the lookahead point strdist away
    # i: the segment of the path we are in
    def genpoint(self, end_path, base, dist):
        seglen = self.dis(end_path, base)
        segv = [end_path[0] - base[0], end_path[1] - base[1]]
        return [base + (dist / seglen) * segv[0], base[1] + (dist / seglen) * segv[1]]

    # projectbase: returns the point on the path with shortest distance to current location
    # cur: current position coordinates
    # i: the segment of the  path we are in
    def projectbase(self, cur_pos, i=0):

        # If we've reached the end, drive toward the last point
        if i == self.n - 1:
            return self.path[self.n - 1]

        # Vector, vector length of current path segment
        segv = [self.path[i + 1][0] - self.path[i][0], self.path[i + 1][1] - self.path[i][1]]
        seglen = self.dis(self.path[i + 1], self.path[i]) * 1.0

        # Vector of current position along current segment
        curv = [cur_pos[0] - self.path[i][0], cur_pos[1] - self.path[i][1]]

        # Distance we've already travelled on current line segment
        curlen = np.dot(curv, segv) / seglen

        # Check if we've passed the end of the line segment
        if curlen <= seglen:
            self.i = i
            return [self.path[i][0] + (curlen / seglen) * segv[0], self.path[i][1] + (curlen / seglen) * segv[1]]
        else:
            # Not on this line segment, check the next
            return self.projectbase(cur_pos, i+1)

    # updatestrdist: returns the new length of the distance between look ahead and current location
    # cur: current location
    # i: the segment of path we are in
    def updatestrdist(self, i):
        return 1 + self.dis(self.cur, self.path[i+1]) # 1 random number here

    # gentarget: returns the coordinates of the target
    # cur: the current position coordinates
    # i: the segment of path we are in
    def gentarget(self, cur_pos, i):

        # Get closest point on path to current point - only check path
        # segments that we haven't traversed yet
        base = self.projectbase(cur_pos, i)

        # Initial lookahead distance
        cur_lookahead_dist = self.lookahead_dist

        # initial i
        cur_i = self.i

        # Get remaining distance on the current segment
        disremain = self.dis(base, self.path[i+1])

        while disremain < cur_lookahead_dist:

            # Compute new lookahead dist
            cur_lookahead_dist = cur_lookahead_dist - disremain

            # Update base
            base = self.path[cur_i+1]

            # Update i
            cur_i += 1

            # Compute new remaining distance
            disremain = self.dis(base, self.path[cur_i+1])

            # Check if we've reached the end of the path
            if cur_i == self.n-1:
                cur_lookahead_dist = 0


        return self.genpoint(self.path[cur_i+1], base, cur_lookahead_dist)

    # linear_vel: returns linear velocity
    def linear_velocity(self):
        return self.linear_vel

    # angular_vel: returns the angular velocity
    def angular_vel(self, lookahead, cur):
        # replace with formula
        # theta = 2 * math.acos(self.cur.orientation.w) along (x, y)
        theta = euler_from_quaternion(self.cur.orientation)
        # clean up and double check the formula
        return 2/((np.square(cur[0]-lookahead[0]) + np.square(cur[1] - lookahead[1])) / (self.linear_velocity()*(math.sin(math.radians(theta))*(lookahead[1]-cur[1]) + math.cos(math.radians(theta))*(lookahead[0]-cur[0]))))

    # main function
    def calc(self, cur_pos):

        # Get lookahead point
        lookahead = self.gentarget(cur_pos, self.i)

        # Determine linear and angular velocity to reach that point
        self.twist.angular.x = self.angular_vel(lookahead, self.cur)
        self.twist.linear.x = self.linear_velocity()
        self.send_twist()


pp = PurePursuit(path_topic, current_topic, twist_topic)
pp.calc()

