#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import openravepy as orpy
from copy import deepcopy
import time
from IPython import embed

# Messages
from sensor_msgs.msg import Image

global handles
handles = []

class Markerdata(object):
	
	def __init__(self):
		self.__markers = {}
		self.__frame = None
		self.__tracking_list = []
		self.__environment = 'false'
		self.__reset = False
		self.__tracking_marker = []
		self.__rotation = []
		self.flag = 0
		

	def load_markers_position(self, position, pixel):
		print'loading position and pixel'
		
		p_copy = deepcopy(position)
		pl_copy = deepcopy(pixel)
		key_keep_list = []
		position_list = []
		#keeping track of markers, only 1 moveable object
		if len(self.__markers) > 0:
			#case 1
			if len(self.__markers) == len(p_copy):
			
				counter = 0
				#finding the ones to keep
				for i in self.__markers:
					for j in len(p_copy):
						current_position = self.__markers[i][0][0]
						diffi = current_position - p_copy[j]
						length = np.sqrt( np.power(diffi[0],2) + np.power(diffi[1],2) +np.power(diffi[2],2))
						key_list.append[i]
						if length <=-0.01 and length >= 0.01:
							key_keep_list.append(i)
							position_list.append(j)
				#updating the makers
				for k in self.__markers:
					for a in len(p_copy):
						if k not in key_keep_list and a not in position_list:
							self.__markers[k][0][0] = p_copy[a]
							self.__markers[k][0][1] = pl_copy[a]
							self.__reset = False
							counter +=1
				if counter >1:
					p_list = range(0, len(p_copy))
					for i in p_list:
						self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
						self._reset = True
			#case 2
			elif len(self.__markers) < len(p_copy):
				print 'loading all markers'
				self.__markers = {}
				p_list = range(0, len(p_copy))
				for i in p_list:
					self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
				self.__reset = True
			
			#case 3	
			elif len(self.__markers) > len(p_copy):
				print'loading markers '
				for i in self.__markers:
					for j in len(p_copy):
						current_position = self.__markers[i][0][0]
						diffi = current_position - p_copy[j]
						length = np.sqrt( np.power(diffi[0],2) + np.power(diffi[1],2) +np.power(diffi[2],2))
						key_list.append[i]
						if length <=-0.01 and length >= 0.01:
							key_keep_list.append(i)
							position_list.append(j)
				diff = len(self.__markers) - len(key_keep_list)
				# multiple blank
				if diff >2:
					self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
					self.__reset = True
				# 1 blank and tracking 1 marker	
				elif diff ==2:
					for k in self.__markers:
						for a in len(p_copy):
							if k not in key_keep_list and a not in position_list:
								if self.__markers[k][0][0] !=0:
									self.__markers[k][0][0] = p_copy[a]
									self.__markers[k][0][1] = pl_copy[a]
									self.__reset = False
								
				#removing 1 position and pixel	
				elif diff ==1:
					for k in self.__markers:
						if k not in key_keep_list:
								self.__markers[k][0][0] = []
								self.__markers[k][0][1] = []
								self.__reset = False
						
				
							
		else:
			p_list = range(0, len(p_copy))
			for i in p_list:
				self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
				self.__reset = True
				
	def load_environment(self):
		if self.__environment == 'false':
			print ' generating environment '
			embed()
			env = orpy.Environment()
			env.Load('/home/goh/catkin_ws/src/osr_course_pkgs/osr_openrave/worlds/just_robot.xml')
			env.SetDefaultViewer()
			#env.SetViewer('qtcoin')
			
			#generate a plane
			
			marker1 = self.__markers[1][0][0]
			marker2 = self.__markers[2][0][0]
			marker3 = self.__markers[3][0][0]
			v1 = marker2 - marker1
			v2 = marker3 - marker1
			markerx = marker1[0]
			markery = marker1[1]
			markerz = marker1[2]
			'''
			v1 = np.array([7,5,3])
			v2 = np.array([-8,-3,2])
			markerx = 0
			markery = 0
			markerz = 0
			'''
			nv = np.cross(v2,v1)
			x = nv[0]
			y = nv[1]
			z = nv[2]
			xy_L = np.sqrt(np.power(x,2) + np.power(y,2))
			anglefromX = np.arctan2(y,x)
			anglefromZ = np.arctan2(xy_L,z)
			absoluteZ = np.absolute(anglefromZ)
			Ry = orpy.matrixFromAxisAngle(np.array((0,1,0)),absoluteZ)
			Rz = orpy.matrixFromAxisAngle(np.array((0,0,1)),anglefromX)
			R = np.dot(Rz,Ry)
			z_shift = np.array([0,0,0.01])
			Rp_shift = np.dot(R[:3,:3],z_shift)
			
			R[0,3] = markerx - Rp_shift[0]
			R[1,3] = markery - Rp_shift[1]
			R[2,3] = markerz - Rp_shift[2]
			Ic = np.zeros((100,100,4))
			Ic[:,:,1] = 10
			Ic[:,:,2] = 10
			Ic[:,:,3] = 10
			
			handles.append(env.drawplane(transform = R,extents=[1.0,2.0],texture=Ic))
			self.__rotation = deepcopy(R)
			#set robot position
			robot = env.GetRobot('robot')
			T = robot.GetTransform()
			rotationZ = orpy.matrixFromAxisAngle(np.array((0,0,1)),np.pi/2)
			p = np.array([1,0,0])
			p_shift = np.dot(R[:3,:3],p)
			T[:3,:3] = R[:3,:3]
			T[0,3] = R[0,3] - p_shift[0]
			T[1,3] = R[1,3] - p_shift[1]
			T[2,3] = R[2,3] - p_shift[2]
	
			robot.SetTransform(T)
			
			self.__environment = 'true'
			print 'Environment generated '
			
		else:
			print'Environment has been generated '
	
	
	
	def load_image (self, frame):
		self.__frame = deepcopy(frame)
		print' loading image '
		font = cv2.FONT_HERSHEY_SIMPLEX
		for i in self.__markers:
			x = int(self.__markers[i][0][1][0])
			y = int(self.__markers[i][0][1][1])
			cv2.putText(self.__frame, str(i), (x,y), font, 1.0, (0, 255, 0), 2)
			cv2.circle(self.__frame,(x,y), 1, (0,0,255), -1)
		cv2.imshow('tracking_image', self.__frame)
	
	
	def marker_tracking(self):
		#user choose the marker for track
		print'Starting marker tracking'
		if self.__reset == True:
			self.__tracking_list = []
			self.__tracking_marker = []
			self.__reset = False
			print 'Reset tracking '
			print 'press any key to continue. '
			cv2.waitKey(0)
		else:
			if self.__tracking_marker == 0:
				self.__tracking_marker = input('Choose marker to track: ')
				self.__tracking_list = []
				print ('Starting tracking of ', self.__tracking_marker)
			if len(self.__tracking_list) != 0 and self.__tracking_marker == True:
		
				print ('Tracking of ', self.__tracking_marker)
				print 'tracking list updated '
				current_position = self.__markers[self.__tracking_marker][0][0]
				diffi = current_position - self.__tracking_list[len(self.__tracking_list)-1]
				length = np.sqrt( np.power(diffi[0],2) + np.power(diffi[1],2) +np.power(diffi[2],2))
				if length <=-0.01 and length >= 0.01:	
					self.__tracking_list.append(self.__markers[self.__tracking_marker][0][0])
			
			else:
				if self.__tracking_marker == True:
					print ('Tracking of ', self.__tracking_marker)
					print 'tracking list updated '
					self.__tracking_list.append(self.__markers[self.__tracking_marker][0][0])
				
			
	def retrieve_marker_path (self):
		print 'retrieving path. '
		self.__tracking_marker = []
		return deepcopy(self.__tracking_list)
		
	def retrieve_markers_data(self):
		print' retrieving data'
		return deepcopy(self.__markers)
		
	def load_rotation (self, R):
		self.__rotation = deepcopy(R)
		
	def get_rotation (self, R):
		R = deepcopy(self.__rotation)
		
	def clear_data(self):
		self.__markers = {}
		self.__frame 
		self.__tracking_list = []
		self.__environment = 'false'
		self.__reset = False
		self.__no_change = False
		self.__tracking_marker = []
		self.__rotation = []
		print 'data clear '
		
	#not used	
	def getVideo(self) :
		rospy.Rate(1)
		self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback )
		print 'Getting Video Feed . . . ' 
		if (self.flag == 1):
			print 'return'
			return
		try:
			rospy.spin()
		except KeyboardInterrupt :
			print "Shutting Down"
	#not used
	def callback(self, data) :
		frame = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
		x.load_image(frame)
		print 'got image '
		self.flag = 1
		cv2.waitKey(30)
		self.image_sub.unregister()
		


def image_callback(data, subscriber):
	global get_image
	image_buffer = 0
	frame = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
	begin = rospy.get_time()
	now = rospy.get_time()
	timeset = 0.0
	print subscriber
	while True:
		if ((now-begin) > timeset):
			break
		else:
			now = rospy.get_time()
	
	#cv2.imshow('image', frame)
	x.load_image(frame)
	get_image = True
	print 'got image '
	cv2.waitKey(30)
	subscriber.unregister()
	#cv2.destroyAllWindows()
	




if "__main__" == __name__:
	rospy.init_node("Datamanagement", anonymous=True)
	x = Markerdata()
	global get_image
	get_image = False
	bridge = CvBridge()
	pixel_list = np.array([[100,200],[300,400],[200,400],[300,100]])
	point_list = np.array([[1,3,2],[3,4,2],[3,5,1],[4,2,3]])
	x.load_markers_position(point_list,pixel_list)
	
	msg = rospy.wait_for_message("/camera/rgb/image_rect_color", Image)
	frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
	x.load_image(frame)
	cv2.waitKey(30)
	#x.getVideo()
	print'stop'
	embed()
	
	x.load_environment()
	x.marker_tracking()
	
	
	
