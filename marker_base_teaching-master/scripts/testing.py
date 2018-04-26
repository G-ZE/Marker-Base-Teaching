#!/usr/bin/env python

# Perception
import rospy
import numpy as np
import openravepy as orpy
from copy import deepcopy
import time
from IPython import embed



if "__main__" == __name__:
	try:
		print "Kinect openrave Start"
		rospy.init_node("testing", anonymous=True)
		env = orpy.Environment()
		env.SetViewer('qtcoin')
		embed()
	except rospy.ROSInterruptException:
		pass
