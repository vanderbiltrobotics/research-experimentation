

# Import ROS packages
import rospy
from geometry_msgs.msg import Twist, Pose, PointStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

# Import other required packages
import numpy as np
from math import sin, cos, pi


class PurePursuit:

    def __init__(self, max_lin_vel, max_lookahead):

        # Set initial linear velocity and lookahead distance
        self.lin_vel = max_lin_vel
        self.lin_vel_dir= 1
        self.lookahead = max_lookahead

        # Initialize path
        self.path = None
        self.path_len = 0
        self.goal_tolerance = 0.03

        # Create subscribers to pose, path topics
        self.pose_sub = rospy.Subscriber("cur_pose", Pose, self.compute_new_twist)
        self.path_sub = rospy.Subscriber("cur_path", Path, self.update_path)

        # Create publisher to twist topic
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.goal_pub = rospy.Publisher("goal_pose", PointStamped, queue_size=0)

    # Callback function for Path topic
    def update_path(self, new_path):

        # Extract x and y coord from each pose in path
        path = [[p.pose.position.x, p.pose.position.y] for p in new_path.poses]

        # Convert to numpy array and store in self.path
        self.path = np.array(path)
        self.path_len = len(self.path)


    # Returns angular velocity required to drive to target point given
    # current pose and linear velocity
    def compute_ang_vel(self, goal_pos, robot_pos, theta):

        # Store coords in variables
        xg = goal_pos[0]
        yg = goal_pos[1]
        xr = robot_pos[0]
        yr = robot_pos[1]

        theta = -((pi / 2) - theta)

        # Convert goal point into vehicle coordinates
        xg_v = (xg - xr) * cos(theta) + (yg - yr) * sin(theta)
        yg_v = - (xg - xr) * sin(theta) + (yg - yr) * cos(theta)

        # Compute numerator and denominator
        curvature = (-2 * xg_v) / (xg_v**2 + yg_v**2)

        # If yg_v negative, drive backwards instead
        self.lin_vel_dir = -1.0 if yg_v < 0.0 else 1.0

        # Return final result
        return self.lin_vel * self.lin_vel_dir * curvature

    # main function
    def compute_new_twist(self, new_pose):

        # Only move if we have a path to follow
        if self.path_len > 0:

            # Extract information from pose message
            position = np.array([new_pose.position.x, new_pose.position.y])
            quat = (
                new_pose.orientation.x,
                new_pose.orientation.y,
                new_pose.orientation.z,
                new_pose.orientation.w
            )
            rpy = euler_from_quaternion(quat)
            theta = rpy[2]

            # Compute distance from current point to each path point
            D = np.linalg.norm(position - self.path, axis=1)

            # Get index of closest point
            nearest = np.argmin(D)
            idx = nearest

            # Search along path until a point found that's farther than lookahead_dist
            while idx < (self.path_len-1) and D[idx] < self.lookahead:
                idx += 1
            goal = self.path[idx]

            # Set velocity and lookahead
            # TODO:
            #       never go negative without abs
            #       linear vel on lookahead and that on variance
            #       scale values

            # Get the path segment
            path_segment = goal - self.path[nearest]

            # Loop over the path and store distances of each point from the vector
            varianc = []
            for i in range(idx-nearest+1):
                varianc.append(np.linalg.norm(self.path[idx + i]-path_segment))

            # Calculate the variance of the data
            variance = np.var(varianc)

            # Calculate lookahead and linear velocity
            self.lookahead = abs(max_lookahead + variance*(0.02 if variance <= 0.0025 else -0.1))
            self.lin_vel = abs(0.1 + (self.lookahead-max_lookahead)*(2.0e+03))
            print variance, self.lin_vel, self.lin_vel_dir, self.lookahead, self.lookahead-max_lookahead

            # Calculate angular vel. from target point and linear vel
            ang_vel = self.compute_ang_vel(goal, position, theta)

            # Create new twist message
            new_twist = Twist()
            new_twist.linear.x = self.lin_vel * self.lin_vel_dir
            new_twist.angular.z = ang_vel

            # Create pose message for current goal position
            goal_msg = PointStamped()
            goal_msg.header.frame_id = "world"
            goal_msg.point.x = goal[0]
            goal_msg.point.y = goal[1]

            # Publish message
            self.twist_pub.publish(new_twist)
            self.goal_pub.publish(goal_msg)

            # If close enough to final point, path is finished
            if np.linalg.norm(position - self.path[-1]) < self.goal_tolerance:
                print "Reached the end of the path"
                self.path = None
                self.path_len = 0

        # If no path, make sure speed is 0
        else:
            self.twist_pub.publish(Twist())


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("pure_pursuit")

    # Read parameters off server
    lin_vel = rospy.get_param("pp_base_lin_vel", default=0.1)
    max_lookahead = rospy.get_param("pp_max_lookahead", default=0.05)

    # Create pure pursuit object
    pp = PurePursuit(lin_vel, max_lookahead)

    # Spin indefinitely
    rospy.spin()

