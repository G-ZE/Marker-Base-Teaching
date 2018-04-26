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
		self.__tracking_marker2 = 0
		self.__markers2 = {}
		self.__start_track = True
		self.__rotation = []
		self.__kin_markers = []
		self.__env = []
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
				print ' case 1 '
				counter = 0
				#finding the ones to keep
				for i in self.__markers:
					for j in range(0, len(p_copy)):
						current_position = self.__markers[i][0][0]
						diffix = current_position[0] - p_copy[j][0]
						diffiy = current_position[1] - p_copy[j][1]
						diffiz = current_position[2] - p_copy[j][2]
						length = np.sqrt( np.power(diffix,2) + np.power(diffiy,2) +np.power(diffiz,2))
						#print length, 'stored x and y ',pl_copy[j][0],pl_copy[j][1]
						if length >=-0.01 and length <= 0.01:
							key_keep_list.append(i)
							position_list.append(j)
				#updating the makers
				for k in self.__markers:
					for a in range(0, len(p_copy)):
						if k not in key_keep_list and a not in position_list:
							self.__markers[k][0][0] = p_copy[a]
							self.__markers[k][0][1] = pl_copy[a]
							self.__reset = False
							self.__tracking_marker2 = k
							counter +=1
				if counter >1:
					print 'reset'
					self.__markers = {}
					p_list = range(0, len(p_copy))
					for i in p_list:
						self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
						self._reset = True
						self.__start_track = True
					self.__markers2 = deepcopy(self.__markers)
				elif counter==1:
					print ' 1 point change '
					self.__start_track = True
					
				elif counter ==0:
					print ' no point change '
					self.__start_track = False
			#case 2
			elif len(self.__markers) < len(p_copy):
				print 'loading all markers, case 2'
				self.__markers = {}
				p_list = range(0, len(p_copy))
				for i in p_list:
					self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
				self.__reset = True
				self.__markers2 = deepcopy(self.__markers)
			
			#case 3	
			elif len(self.__markers) > len(p_copy):
				current_marker_numbers = 0
				print'loading markers, case 3'
				for i in self.__markers:
					for j in range(0, len(p_copy)):
						current_position = self.__markers[i][0][0]
						diffix = current_position[0] - p_copy[j][0]
						diffiy = current_position[1] - p_copy[j][1]
						diffiz = current_position[2] - p_copy[j][2]
						length = np.sqrt( np.power(diffix,2) + np.power(diffiy,2) +np.power(diffiz,2))
						
						if length >=-0.01 and length <= 0.01:
							key_keep_list.append(i)
							position_list.append(j)
				Check_diff = len(self.__markers) - len(p_copy)
				
				# To check for blanks			
				for i in self.__markers:			
					if self.__markers[i][0][0][0] !=0:
						current_marker_numbers +=1
				blanks = len(self.__markers) - current_marker_numbers
				diff = current_marker_numbers - len(p_copy)
				print diff
				# multiple blank
				if diff >=2:
					print ' Do nothing '
					
					'''
					print 'reset '
					self.__markers = {}
					p_list = range(0, len(p_copy))
					for i in p_list:
						self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
					self.__reset = True
					self.__markers2 = deepcopy(self.__markers)
					'''
				# 1 blank and tracking 1 marker	
				elif diff ==0:
					print' tracking with 1 blank '
					for k in self.__markers:
						for a in range(0, len(p_copy)):
							if k not in key_keep_list and a not in position_list:
								if self.__markers[k][0][0][0] !=0:
									self.__markers[k][0][0] = p_copy[a]
									self.__markers[k][0][1] = pl_copy[a]
									self.__reset = False
									self.__tracking_marker2 = k
								
				#removing 1 position and pixel	
				elif diff ==1:
					print ' remove point '
					for k in self.__markers:
						if k not in key_keep_list:
								#position
								self.__markers[k][0][0][0] = 0
								self.__markers[k][0][0][1] = 0
								self.__markers[k][0][0][2] = 0
								#pixel
								self.__markers[k][0][1][0] = 0
								self.__markers[k][0][1][1] = 0
								self.__reset = False
								self.__start_track = False
								self.__tracking_marker2 = k
						
				
							
		else:
			p_list = range(0, len(p_copy))
			for i in p_list:
				self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
			self.__reset = True
			self.__start_track = True
			self.__markers2 = deepcopy(self.__markers)
				
	def load_environment(self, actual_points):
		
		def create_box(T, color = [0, 0.6, 0]):
			box = orpy.RaveCreateKinBody(self.__env, '')
			box.SetName('box')
			box.InitFromBoxes(np.array([[0,0,0,0.01,0.01,0.005]]), True)    
			g = box.GetLinks()[0].GetGeometries()[0]
			g.SetAmbientColor(color)
			g.SetDiffuseColor(color)
			box.SetTransform(T)
			self.__env.Add(box,True)
			return box
		
		if self.__environment == 'false':
			print ' generating environment '
			self.__env = orpy.Environment()
			self.__env.Load('/home/goh/catkin_ws/src/osr_course_pkgs/osr_openrave/worlds/just_robot.xml')
			self.__env.SetDefaultViewer()
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
			self.__rotation = deepcopy(R)
			z_shift = np.array([0,0,0.01])
			Rp_shift = np.dot(R[:3,:3],z_shift)
			
			R[0,3] = markerx - Rp_shift[0]
			R[1,3] = markery - Rp_shift[1]
			R[2,3] = markerz - Rp_shift[2]
			Ic = np.zeros((100,100,4))
			Ic[:,:,1] = 10
			Ic[:,:,2] = 10
			Ic[:,:,3] = 10
			with self.__env:
				handles.append(self.__env.drawplane(transform = R,extents=[1.0,2.0],texture=Ic))
			
			#set robot position
			robot = self.__env.GetRobot('robot')
			T = robot.GetTransform()
			rotationZ = orpy.matrixFromAxisAngle(np.array((0,0,1)),np.pi/2)
			p = np.array([1,0,0])
			p_shift = np.dot(R[:3,:3],p)
			T[:3,:3] = R[:3,:3]
			T[0,3] = R[0,3] - p_shift[0]
			T[1,3] = R[1,3] - p_shift[1]
			T[2,3] = R[2,3] - p_shift[2]
			with self.__env:
				robot.SetTransform(T)
			self.__environment = 'true'	
			print 'Environment generated '
			
		else:
			print'Environment has been generated '
		print'Generating markers '
		#creating markers
		newkin_markers = []
		if len(actual_points) > 0 and len(self.__kin_markers) == 0:
			# first time detected
			print " creating markers"
			for i in range(len(actual_points)):
				T = np.eye(4)	
				container_center = np.array([actual_points[i][0], actual_points[i][1], actual_points[i][2]])
				T[:3, 3] = container_center
				#R = []
				#R = dm.get_rotation()
				T[:3,:3] = R[:3,:3]
				kin_marker = create_box(T, color = [0, 0, 0.6])
				self.__kin_markers.append(kin_marker)
					
		elif len(actual_points)> 0 and len(self.__kin_markers) !=0:
			# second and more detected
			# creating new markers and remove markers that are not there
			# old points in actualpoints
			print " keeping existing markers"
			for i in range(len(self.__kin_markers)):
				for j in range(len(actual_points)):
					old_marker = self.__kin_markers[i].GetTransform()[:3,3]
					dist = np.sqrt(np.square(old_marker[0]) + np.square(old_marker[1]) + np.square(old_marker[2]))
					dist2 = np.sqrt(np.square(actual_points[j][0]) + np.square(actual_points[j][1]) + np.square(actual_points[j][2]))
					if (dist-dist2) == 0.01:
						newkin_markers.append(self.__kin_markers[i])
						self.__kin_markers.Remove(self.__kin_markers[i])
						actual_points.Remove(actual_points[j])
				
			print " creating new markers and remove non markers"
			for i in range(len(self.__kin_markers)):
				with self.__env:
					self.__env.Remove(self.__kin_markers[i])
			for j in range(len(actual_points)):
				#new markers
				T = np.eye(4)
				container_center = np.array([actual_points[j][0], actual_points[j][1], actual_points[j][2]])
				T[:3, 3] = container_center
				#R = []
				R = deepcopy(self.__rotation)
				T[:3,:3] = R[:3,:3]
				kin_marker = create_box(T, color = [0, 0, 0.6])
				with self.__env:
					if self.__env.CheckCollision(kin_marker):
						self.__env.Remove(kin_marker)	
					else:
						newkin_markers.append(kin_marker)

						
			self.__kin_markers = newkin_markers
	
	
	
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
	
	#outdated
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
				if length !=0:	
					self.__tracking_list.append(self.__markers[self.__tracking_marker][0][0])
			else:
				if self.__tracking_marker == True:
					print ('Tracking of ', self.__tracking_marker)
					print 'tracking list updated '
					self.__tracking_list.append(self.__markers[self.__tracking_marker][0][0])
				
	def marker_tracking2(self):
		#track and store the moved marker
		print'Starting marker tracking'
		if self.__start_track = True:
			if self.__reset == True:
				self.__tracking_list = []
				self.__tracking_marker = []
				self.__tracking_marker2 = 0
				self.__reset = False
				print 'Reset tracking '
				print 'press any key to continue. '
				cv2.waitKey(0)
			else:
				if self.__tracking_marker2 != 0:
					# track if there is a postion
					if self.__markers[self.__tracking_marker2][0][0][0] != 0:
						print ('Tracking of ', self.__tracking_marker2)
						#add only if different from last point
						current_position = self.__markers[self.__tracking_marker2][0][0]
						diffix = current_position[0] - self.__markers2[self.__tracking_marker2][len(self.__markers2[self.__tracking_marker2])-1][0][0]
						diffiy = current_position[1] - self.__markers2[self.__tracking_marker2][len(self.__markers2[self.__tracking_marker2])-1][0][1]
						diffiz = current_position[2] - self.__markers2[self.__tracking_marker2][len(self.__markers2[self.__tracking_marker2])-1][0][2]
						length = np.sqrt( np.power(diffix,2) + np.power(diffiy,2) +np.power(diffiz,2))
						if length >=0.01 or length <=-0.01:
							print 'tracking list updated '
							self.__markers2[self.__tracking_marker2].append(self.__markers[self.__tracking_marker2][0])
						else:
							print 'length too small', length
					else:
						print 'nothing to track'
				else:
					print ' no marker selected '
		else:
			print ' not tracking'
			
	def retrieve_marker_path (self):
		print ('retrieving path of ',self.__tracking_marker)
		self.__tracking_list = []
		for i in len(self.__markers2[self.__tracking_marker]):
			self.__tracking_list.append(self.__markers2[self.__tracking_marker][i][0])
		self.__tracking_marker = []
		return deepcopy(self.__tracking_list)
		
	def retrieve_markers_data(self):
		print' retrieving markerset 1'
		return deepcopy(self.__markers)
		
	def retrieve_markers_data2(self):
		print' retrieving markerset 2'
		return deepcopy(self.__markers2)
		
	def load_rotation (self, R):
		self.__rotation = deepcopy(R)
		
	def get_rotation (self):
		return deepcopy(self.__rotation)
		
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
	
	
	
