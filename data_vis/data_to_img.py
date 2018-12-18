
# Import required packages
from matplotlib import pyplot as plt
import numpy as np
import cv2


# Arena config parameters
scale = 1.2
field_width = 369
field_height = 576
dig_zone_line = 192
obs_zone_line = 384
bin_width = 48
bin_height = 165
bin_offset = 50
bin_config = 'left'

# Robot config parameters
robot_width = 60
robot_height = 80

# ----------------- #
# CREATE BASE IMAGE #
# ----------------- #

# Initialize full image
base_img = np.zeros((int(field_height*scale + 150*scale), int(field_width*scale), 3), 'uint8') + 255

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

# Sample robot pose

# tf quaternion, getRPY, Y is what we want


# Save image for reference
cv2.imwrite('map_img.bmp', base_img)