#!/usr/bin/env python
""" Docstring """
# Standard libraries
import glob
import copy
import numpy as np
import threading
import time
from scipy import misc
# ROS libraries
from geometry_msgs.msg import Pose2D
import rospy
# Local libraries
from resources import saveposes
import videosensor

# Connect and configure the camera
conf_file = "./resources/config/video_sensor1.cfg"
camera = videosensor.camera_startup(conf_file)
image, _ = videosensor.set_tracker(camera)
# Print the image
misc.imshow(image.image)
#Should wait around 1 second for configuring the camera.
# Instructions for getting the location obtained by the localization algorithm.
locations = camera.get_register('ACTUAL_LOCATION')['1']
contours = np.array(locations) / camera._scale
tmp = np.copy(contours[:,0])
contours[:,0] = contours[:,1]
contours[:,1] = tmp
image.contours = [contours]
image.correct_distortion()
shapes = image.get_shapes(get_contours=False)
# in 't' is stored the Triangle object representing the detected UGV.
t = shapes[0]
t.local2global(camera.offsets, K=4)
t.homography(camera._H)


