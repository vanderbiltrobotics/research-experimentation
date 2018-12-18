
# Import required packages
from matplotlib import pyplot as plt
import numpy as np
import cv2
from geometry_msgs.msg import Pose, Twist
import tf.transformations
from math import sin, cos

# Arena config parameters
scale = 2
field_width = 369
field_height = 576
dig_zone_line = 192
obs_zone_line = 384
bin_width = 48
bin_height = 165
bin_offset = 50
bin_config = 'left'

# Robot config parameters
robot_width = .60
robot_height = .80

# ----------------- #
# CREATE BASE IMAGE #
# ----------------- #

# Initialize full image
base_img = np.zeros((int(field_height*scale), int(field_width*scale), 3), 'uint8') + 255

# Grey region for competition area
base_img[0: int(576*scale), :, :] = 245

# Add horizontal lines demarcating regions
cv2.line(base_img, (0, int(dig_zone_line*scale)), (int(field_width*scale), int(dig_zone_line*scale)), (175, 175, 175), 1)
cv2.line(base_img, (0, int(obs_zone_line*scale)), (int(field_width*scale), int(obs_zone_line*scale)), (175, 175, 175), 1)

# Calculate corner coords for collection bin
pt1_x = 0
pt1_y = (field_height - bin_offset) * scale
pt2_y = pt1_y - (bin_height * scale)

# Adjust for field configuration
if bin_config == 'right':
    pt1_x += (field_width - bin_width) * scale
pt2_x = pt1_x + (bin_width * scale)

# Draw collection bin
cv2.rectangle(base_img, (int(pt1_x), int(pt1_y)), (int(pt2_x), int(pt2_y)), (255, 231, 198), cv2.FILLED)
cv2.rectangle(base_img, (int(pt1_x), int(pt1_y)), (int(pt2_x), int(pt2_y)), (160, 122, 69), 2)


# ----------------------- #
# ADD ROBOT POSE TO IMAGE #
# ----------------------- #

# Sample robot pose - (positions are in meters)
sample_pose = Pose()
sample_pose.position.x = 1.0
sample_pose.position.y = 2.0
sample_pose.position.z = 0.0
sample_pose.orientation.x = 0
sample_pose.orientation.y = 0
sample_pose.orientation.z = 0.383
sample_pose.orientation.w = 0.924

# Get quaternion from pose message
quat = [sample_pose.orientation.x, sample_pose.orientation.y, sample_pose.orientation.z, sample_pose.orientation.w]

# Extract yaw - this is the rotation in xy plane
theta = tf.transformations.euler_from_quaternion(quat)[2]

# Draw arrow representing robot pose
pt1_x = sample_pose.position.x
pt1_y = sample_pose.position.y
pt2_x = pt1_x + (cos(theta) * .35)
pt2_y = pt1_y - (sin(theta) * .35)
cv2.arrowedLine(base_img, (int(pt1_x*100*scale), int(pt1_y*100*scale)), (int(pt2_x*100*scale), int(pt2_y*100*scale)), (0, 150, 0), 2, tipLength=0.15)

# Compute corners of robot before rotation & translation
fr = [robot_height / 2.0, robot_width / 2.0]
fl = [robot_height / 2.0, -robot_width / 2.0]
br = [-robot_height / 2.0, robot_width / 2.0]
bl = [-robot_height / 2.0, -robot_width / 2.0]
corners = np.array([fr, fl, br, bl]).transpose()

# Rotation matrix
R = tf.transformations.rotation_matrix(theta, (0, 0, 1))[0:2, 0:2]

# Compute new corner coords
new_corners = np.matmul(R, corners)

# Translate
new_corners[0, :] = new_corners[0, :] + pt1_x
new_corners[1, :] = pt1_y - new_corners[1, :]
new_corners = (new_corners * 100 * scale).astype(np.int32)

# Draw edges on map
cv2.line(base_img, (new_corners[0, 0], new_corners[1, 0]), (new_corners[0, 1], new_corners[1, 1]), (0,0,0), 2)
cv2.line(base_img, (new_corners[0, 1], new_corners[1, 1]), (new_corners[0, 3], new_corners[1, 3]), (0,0,0), 2)
cv2.line(base_img, (new_corners[0, 0], new_corners[1, 0]), (new_corners[0, 2], new_corners[1, 2]), (0,0,0), 2)
cv2.line(base_img, (new_corners[0, 2], new_corners[1, 2]), (new_corners[0, 3], new_corners[1, 3]), (0,0,0), 2)


# sample robot Twist
sample_twist = Twist()
sample_twist.linear.x = 0.4
sample_twist.angular.z = 3.142 / 4

# generate arc of points robot will traverse in next second if these speeds are maintained
dt = 0.1
elapsed = 0.0
total_time = 2.0
prev_point = [pt1_x, pt1_y]
prev_theta = theta
while elapsed < total_time:
    avg_theta = prev_theta + (0.5 * sample_twist.angular.z * dt)
    new_x = prev_point[0] + (sample_twist.linear.x * cos(avg_theta) * dt)
    new_y = prev_point[1] - (sample_twist.linear.x * sin(avg_theta) * dt)

    # translated, scaled points for drawing
    prev_x_plot = int(prev_point[0] * 100 * scale)
    prev_y_plot = int(prev_point[1] * 100 * scale)
    new_x_plot = int(new_x * 100 * scale)
    new_y_plot = int(new_y * 100 * scale)

    # add arc segment to drawing
    cv2.line(base_img, (prev_x_plot, prev_y_plot), (new_x_plot, new_y_plot), (0, 0, 150), 2)

    # Update variables
    elapsed += dt
    prev_theta = prev_theta + (avg_theta - prev_theta) * 2
    prev_point = [new_x, new_y]



# Save image for reference
cv2.imwrite('map_img.bmp', base_img)

