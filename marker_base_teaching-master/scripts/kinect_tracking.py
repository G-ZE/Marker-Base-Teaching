#!/usr/bin/env python

# Perception
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import openravepy as orpy
from copy import deepcopy
import time
import threading
import sys
from IPython import embed
import Datamanagement
import yaml
# Messages
from marker_base_teaching.msg import markers
from sensor_msgs.msg import Image
'''
			start = rospy.get_rostime()
			print "getting points"
			sub_once = rospy.Subscriber("kinect_openrave_marker_points", markers, points_callback, queue_size =1)
			now = rospy.get_rostime()
			while (now.secs - start.secs) <timeset:
				now = rospy.get_rostime()
				
			from IPython import embed
			embed()
	
'''	
				




def imageandpoints_callback(msg):

	global get
	get = False
	global publish_image
	publish_image = False
	#print "calling points "
	newpoints = []
	newpoints2 = []
	actual_points = []
	pixel_points = []
	all_points = []
	allpoints3 = []
	all_points2 = []
	startseconds = rospy.get_time()
	if msg !=0:
		for i in range(len(msg.arraypoints)):
			msgpoints =[msg.arraypoints[i]]
			msgpixel = [msg.arraypixelpoints[i]]
			pl = msgpixel[0]
			p = msgpoints[0]
			pixel = np.array([pl.x, pl.y])
			point = np.array([p.x,p.y,p.z])
			newpoints.append(point)
			pixel_points.append(pixel)
		actual_points = deepcopy(newpoints)
		
		
		frame = bridge.imgmsg_to_cv2(msg.image, desired_encoding="passthrough")
		BGR = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)

		#dm.load_markers_position(actual_points,pixel_points)
		#dm.load_image(BGR)
		#Loading the list
		for i in range(len(msg.arraypoints2)):
			msgpoints =[msg.arraypoints2[i]]
			p = msgpoints[0]
			point = np.array([p.x,p.y,p.z])
			newpoints2.append(point)
		all_points = deepcopy(newpoints2)
		
		for i in range(len(msg.allsize)):
			msgallsize = msg.allsize[i]
			allpoints3.append(msgallsize)
		all_points2 = deepcopy(newpoints3)
		list_size = deepcopy(msg.size)
		dm.load_marker_path(list_size,all_points2,all_points)
		#load env and start tracking	
		#dm.load_environment(actual_points)
		#publish_image = dm.marker_tracking2()
	if len(actual_points) !=0:
		print "data retieve from msg "
		get = True
		endseconds = rospy.get_time()
		cycletime = endseconds - startseconds
		print "time for 1 cycle", cycletime
#not used
def image_callback(data):
	global tracker_sub_once
	global get_image
	image_buffer = 0
	frame = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
	#dm.load_image(frame)
	get_image = True
	print "got image "
	font = cv2.FONT_HERSHEY_SIMPLEX
	c_markers = dm.retrieve_markers_data()
	for i in c_markers:
		x = int(c_markers[i][0][1][0])
		y = int(c_markers[i][0][1][1])
		cv2.putText(frame, str(i), (x,y), font, 1.0, (0, 255, 0), 2)
		cv2.circle(frame,(x,y), 1, (0,0,255), -1)
	cv2.imshow("tracking_image", frame)
	cv2.waitKey(30)
	if cv2.waitKey(30) & 0xFF == ord("q"):
		tracker_sub_once.unregister()
		cv2.destroyAllWindows()

#retieving image and points
def background(e,lock,pub,r):
	timeset = 0.0
	global Stopping_event
	Stopping_event = False
	global get
	
	global publish_image
	publish_image = False
	sub_once = rospy.Subscriber("kinect_openrave_marker_points", markers, imageandpoints_callback, queue_size =1)
	while True:
		#geting points
		#startseconds = rospy.get_time()
		if Stopping_event == True:
			event_is_set = e.wait()
			sub_once.unregister()
			if Stopping_event == False:
				e.clear()
				sub_once = rospy.Subscriber("kinect_openrave_marker_points", markers, imageandpoints_callback, queue_size =1)
		#print " retieving markers data "
		get = False
		
		begin = rospy.get_rostime().secs
		currenttime =rospy.get_rostime().secs
		while ((currenttime-begin) < timeset) and get != True:
			currenttime =rospy.get_rostime().secs
		#load env and start tracking	
		#dm.load_environment()
		#with lock:
			#publish_image = dm.marker_tracking2()
		#sub_once.unregister()
		#endseconds = rospy.get_time()
		#cycletime = endseconds - startseconds
		#print "time for 1 cycle", cycletime
		#print " shutdown subscriber "
		
		
		
		
		
#not used
def background2(e,lock, pub):
	timeset = 10.0
	global Stopping_event
	Stopping_event = False
	while True:
		#geting points
		if Stopping_event == True:
			event_is_set = e.wait()
			if Stopping_event == False:
				e.clear()
		while True:
			print " getting points and image"
			msg = rospy.wait_for_message("kinect_openrave_marker_points", markers)
			
			if Stopping_event == True:
				event_is_set = e.wait()
				if Stopping_event == False:
					e.clear()
			
			
			newpoints = []
			pixel_points = []
			for i in range(len(msg.arraypoints)):
				msgpoints =[msg.arraypoints[i]]
				msgpixel = [msg.arraypixelpoints[i]]
				pl = msgpixel[0]
				p = msgpoints[0]
				pixel = np.array([pl.x, pl.y])
				point = np.array([p.x,p.y,p.z])
				newpoints.append(point)
				pixel_points.append(pixel)
				#print 'retrieve pixel points' , pl.x , pl.y
				#print 'retrieve points' , p.x , p.y
				#print msg.arraypixelpoints
			
			actual_points = deepcopy(newpoints)
			dm.load_markers_position(actual_points,pixel_points)
			
			frame = bridge.imgmsg_to_cv2(msg.image, desired_encoding="passthrough")
			BGR = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
			print "got image "
			font = cv2.FONT_HERSHEY_SIMPLEX
			c_markers = dm.retrieve_markers_data()
			for i in c_markers:
				x = int(c_markers[i][0][1][0])
				y = int(c_markers[i][0][1][1])
				print "drawing",(x,y)
				print "number " , i ,"position ", c_markers[i][0][0]
				if x !=0 and y!=0:
					cv2.putText(BGR, str(i), (x,y), font, 1.0, (0, 255, 0), 2)
					cv2.circle(BGR,(x,y), 1, (0,0,255), -1)
				
			#marker1 = c_markers[1][0][0]
			#marker2 = c_markers[2][0][0]
			#marker3 = c_markers[3][0][0]
			#v23 = marker2 - marker3
			#v31 = marker3 - marker1
			#length23 = np.sqrt( np.power(v23[0],2) + np.power(v23[1],2) +np.power(v23[2],2))
			#length31 = np.sqrt( np.power(v31[0],2) + np.power(v31[1],2) +np.power(v31[2],2))
			#print 'length from 2 to 3 ', length23, ' and length from 3 to 1 ',length31
			
			image_message = bridge.cv2_to_imgmsg(BGR, "bgr8")
			pub.publish(image_message)
			#cv2.imshow('tracking_image', BGR)
			#cv2.waitKey(30)
			
			if len(actual_points) > 0:
				break
		
		# loading environment
		#with lock:	
			#dm.load_environment()
		
		#dm.marker_tracking2(lock)

#for publishing the image back
def background3(e,lock, pub,r):
	global Stopping_event
	Stopping_event = False
	global publish_image
	publish_image = False
	while not rospy.is_shutdown():
		
		if Stopping_event == True:
			event_is_set = e.wait()
			if Stopping_event == False:
				e.clear()
		if publish_image == True:
			print " publish image is true"
			with lock:
				frame = dm.get_image()
			if frame is not None:
				print" got frame"
				cv2.imshow('trackingimage', frame)
				cv2.waitKey(30)
				image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
				#pub.publish(image_message)
				publish_image = False
		#r.sleep()


if "__main__" == __name__:
	try:
		print "Kinect tracking Start"
		rospy.init_node("kinect_tracking", anonymous=True)
		dm = Datamanagement.Markerdata()
		#global newpoints
		#global sub_once
		#sub_once = rospy.Subscriber("kinect_openrave_marker_points", markers, points_callback, queue_size =1)
		pub = rospy.Publisher("marker_number", Image, queue_size=1)
		r = rospy.Rate(10)
		global get
		get= False
		global Stopping_event
		Stopping_event = False
		e = threading.Event()
		lock = threading.Lock()
		# get marker data, generate env and track
		print "start thread 1 loading markers, enviorment and tracking"
		threading1 = threading.Thread(target=background,args=(e,lock,pub,r))
		threading1.daemon = True
		threading1.start()
		# publish back to pose estimation to display image
		print "start thread 2 send picture"
		threading2 = threading.Thread(target=background3,args=(e,lock,pub,r))
		threading2.daemon = True
		threading2.start()
		
		
		bridge = CvBridge()
	
		
		while True:	
			
			'''
			while True:
				print " getting points and image"
				msg = rospy.wait_for_message("kinect_openrave_marker_points", markers)
				
				newpoints = []
				pixel_points = []
				for i in range(len(msg.arraypoints)):
					msgpoints =[msg.arraypoints[i]]
					msgpixel = [msg.arraypixelpoints[i]]
					pl = msgpixel[0]
					p = msgpoints[0]
					pixel = np.array([pl.x, pl.y])
					point = np.array([p.x,p.y,p.z])
					newpoints.append(point)
					pixel_points.append(pixel)
					#print 'retrieve pixel points' , pl.x , pl.y
					#print 'retrieve points' , p.x , p.y
					#print msg.arraypixelpoints
				
				actual_points = deepcopy(newpoints)
				dm.load_markers_position(actual_points,pixel_points)
				
				frame = bridge.imgmsg_to_cv2(msg.image, desired_encoding="passthrough")
				BGR = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
				print 'got image '
				font = cv2.FONT_HERSHEY_SIMPLEX
				c_markers = dm.retrieve_markers_data()
				for i in c_markers:
					x = int(c_markers[i][0][1][0])
					y = int(c_markers[i][0][1][1])
					print 'drawing',(x,y)
					print 'number ' , i ,'position ', c_markers[i][0][0]
					if x !=0 and y!=0:
						cv2.putText(BGR, str(i), (x,y), font, 1.0, (0, 255, 0), 2)
						cv2.circle(BGR,(x,y), 1, (0,0,255), -1)
					
				#marker1 = c_markers[1][0][0]
				#marker2 = c_markers[2][0][0]
				#marker3 = c_markers[3][0][0]
				#v23 = marker2 - marker3
				#v31 = marker3 - marker1
				#length23 = np.sqrt( np.power(v23[0],2) + np.power(v23[1],2) +np.power(v23[2],2))
				#length31 = np.sqrt( np.power(v31[0],2) + np.power(v31[1],2) +np.power(v31[2],2))
				#print 'length from 2 to 3 ', length23, ' and length from 3 to 1 ',length31
					
				cv2.imshow('tracking_image', BGR)
				cv2.waitKey(30)
				
				if len(actual_points) > 0:
					break
			
			#loading environment	
			dm.load_environment()
			'''
			
			
			
			
			#dm.marker_tracking()
			#dm.marker_tracking2(lock)
			#embed()
			path =[]
			entered = []
			entered = raw_input("type (done) to retrieve path")
			if entered == "done":
				print " got input "
				Stopping_event = True
				path = dm.retrieve_marker_path(lock)
				with lock:
					tracking_data = dm.retrieve_markers_data2()
					with open("/home/goh/catkin_ws/src/marker_base_teaching-master/kinect_retrieve_saves/tracking_Data.yml","w") as outfile:
						yaml.dump(tracking_data, outfile, default_flow_style=False)
					
			#if cv2.waitKey(30) & 0xFF == ord('d'):
				#path = dm.retrieve_marker_path()
				#embed()
			else:
				print" not retrieving path "
				
			if len(path) !=0:
				move_path = dm.Move_along_path(path,lock)
				if move_path == True:
					Stopping_event = False
					e.set()
				
			
	except rospy.ROSInterruptException:
		pass
