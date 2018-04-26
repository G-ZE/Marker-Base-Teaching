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
		self.__cppmarkers = {}
		self.__markersloaded = False
		self.__frame = None
		self.__tracking_list = []
		self.__environment = False
		self.__reset = False
		self.__tracking_marker = []
		self.__tracking_marker2 = []
		self.__markers2 = {}
		self.__start_track = False
		self.__rotation = np.eye(4)
		self.__kin_markers = []
		self.__actual_points = []
		self.env = None
		self.flag = 0
		

	def load_markers_position(self, position, pixel):
		#for the env
		self.__actual_points = deepcopy(position)
		#print'loading position and pixel'
		p_copy = deepcopy(position)
		pl_copy = deepcopy(pixel)
		key_keep_list = []
		position_list = []
		#keeping track of markers, only 1 moveable object
		if len(self.__markers) > 0:
			#case 1 same number of points
			if len(self.__markers) == len(p_copy):
				#print ' case 1 '
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
							current_position = self.__markers[k][0][0]
							diffix = current_position[0] - p_copy[a][0]
							diffiy = current_position[1] - p_copy[a][1]
							diffiz = current_position[2] - p_copy[a][2]
							length = np.sqrt( np.power(diffix,2) + np.power(diffiy,2) +np.power(diffiz,2))
							print "lenght diff", length
							if length >=-0.05 and length <=0.05:
								self.__markers[k][0][0] = p_copy[a]
								self.__markers[k][0][1] = pl_copy[a]
								self.__reset = False
								self.__tracking_marker2.append(k)
								counter +=1
			
				if counter>=1:
					print ' 1 point change '
					self.__start_track = True
					
				elif counter ==0:
					print ' no point change '
					self.__start_track = False
			#case 2 more new points
			elif len(self.__markers) < len(p_copy):
				#print 'loading all markers, case 2'
				self.__markers = {}
				p_list = range(0, len(p_copy))
				for i in p_list:
					self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
				self.__reset = True
				self.__start_track = True
				self.__markers2 = deepcopy(self.__markers)
			
			#case 3	less new points
			
							
		else:
			p_list = range(0, len(p_copy))
			for i in p_list:
				self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
			self.__reset = True
			self.__start_track = True
			self.__markersloaded = True
			self.__markers2 = deepcopy(self.__markers)
	#not used
	def load_markers_position_v2(self, position, pixel):
		#print'loading position and pixel'
		
		p_copy = deepcopy(position)
		pl_copy = deepcopy(pixel)
		self.__actual_points = deepcopy(position)
		key_keep_list = []
		position_list = []
		#keeping track of markers, only 1 moveable object
		if len(self.__markers) > 0:
			#case 1
			if len(self.__markers) == len(p_copy):
				#print ' case 1 '
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
					#print 'reset'
					self.__markers = {}
					p_list = range(0, len(p_copy))
					for i in p_list:
						self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
						self._reset = True
						self.__start_track = True
					self.__markers2 = deepcopy(self.__markers)
				elif counter==1:
					#print ' 1 point change '
					self.__start_track = True
					
				elif counter ==0:
					#print ' no point change '
					self.__start_track = False
			#case 2
			elif len(self.__markers) < len(p_copy):
				#print 'loading all markers, case 2'
				self.__markers = {}
				p_list = range(0, len(p_copy))
				for i in p_list:
					self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
				self.__reset = True
				self.__start_track = True
				self.__markers2 = deepcopy(self.__markers)
			
			#case 3	not working
			'''
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
					
					
					print 'reset '
					self.__markers = {}
					p_list = range(0, len(p_copy))
					for i in p_list:
						self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
					self.__reset = True
					self.__markers2 = deepcopy(self.__markers)
					
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
						
			'''	
							
		else:
			p_list = range(0, len(p_copy))
			for i in p_list:
				self.__markers[i+1] = [[p_copy[i],pl_copy[i]]]
			self.__reset = True
			self.__start_track = True
			self.__markersloaded = True
			self.__markers2 = deepcopy(self.__markers)
	
	
	def load_environment(self, position):
		self.__actual_points = deepcopy(position)
		if self.__markersloaded == True:
			#print ' preparing to generate environment '
			print ' generating environment '
			self.env = orpy.Environment()
			self.env.Load('/home/goh/catkin_ws/src/osr_course_pkgs/osr_openrave/worlds/just_robot.xml')
			self.env.SetDefaultViewer()
			
			#with lock:
			#number_of_markers = len(self.__markers)
			number_of_markers = len(self.__cppmarkers)
			if self.__environment == False and number_of_markers >=3:
				
				#embed()
				#env.SetViewer('qtcoin')
		
				#generate a plane
				
				marker1 = self.__cppmarkers[1][0]
				marker2 = self.__cppmarkers[2][0]
				marker3 = self.__cppmarkers[3][0]
				
				#marker1 = self.__markers[1][0][0]
				#marker2 = self.__markers[2][0][0]
				#marker3 = self.__markers[3][0][0]
				
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
				with self.env:
					handles.append(self.env.drawplane(transform = R,extents=[1.0,2.0],texture=Ic))
				
				#set robot position
				robot = self.env.GetRobot('robot')
				base = self.env.GetKinBody('denso_base')
				self.env.Remove(base)
				T = robot.GetTransform()
				rotationZ = orpy.matrixFromAxisAngle(np.array((0,0,1)),np.pi/2)
				x_direction = np.array([0.5,0,0])
				x_shift = np.dot(R[:3,:3],x_direction)
				z_direction = np.array([0,0,0.185])
				z_shift = np.dot(R[:3,:3],z_direction)
				T[:3,:3] = R[:3,:3]
				T[0,3] = R[0,3] - x_shift[0]- z_shift[0]
				T[1,3] = R[1,3] - x_shift[1]- z_shift[1]
				T[2,3] = R[2,3] - x_shift[2]- z_shift[2]
				with self.env:
					robot.SetTransform(T)
				self.__environment = True	
				print 'Environment generated '
				
			#else:
				#print'Environment has been generated '
			
			#print'Generating markers '
			#creating markers
			def create_box(T, color = [0, 0.6, 0]):
				box = orpy.RaveCreateKinBody(self.env, '')
				box.SetName('box')
				box.InitFromBoxes(np.array([[0,0,0,0.01,0.01,0.005]]), True)    
				g = box.GetLinks()[0].GetGeometries()[0]
				g.SetAmbientColor(color)
				g.SetDiffuseColor(color)
				box.SetTransform(T)
				self.env.Add(box,True)
				return box
			
			newkin_markers = []
			actual_points = self.__actual_points
			kin_markers = self.__kin_markers
			if len(actual_points) > 0 and len(kin_markers) == 0:
				# first time detected
				#print " creating markers"
				for i in range(len(actual_points)):
					T = np.eye(4)	
					container_center = np.array([actual_points[i][0], actual_points[i][1], actual_points[i][2]])
					T[:3, 3] = container_center
					#R = []
					#R = dm.get_rotation()
					R = deepcopy(self.__rotation)
					T[:3,:3] = R[:3,:3]
					kin_marker = create_box(T, color = [0, 0, 0.6])
					kin_markers.append(kin_marker)
						
			elif len(actual_points)> 0 and len(kin_markers) !=0:
				# second and more detected
				# creating new markers and remove markers that are not there
				# old points in actualpoints
				#print " keeping existing markers"
				for i in range(len(kin_markers)):
					for j in range(len(actual_points)):
						old_marker = kin_markers[i].GetTransform()[:3,3]
						dist = np.sqrt(np.square(old_marker[0]) + np.square(old_marker[1]) + np.square(old_marker[2]))
						dist2 = np.sqrt(np.square(actual_points[j][0]) + np.square(actual_points[j][1]) + np.square(actual_points[j][2]))
						if (dist-dist2) == 0.01:
							newkin_markers.append(kin_markers[i])
							self.__kin_markers.Remove(kin_markers[i])
							self.__actual_points.Remove(actual_points[j])
					
				#print " creating new markers and remove non markers"
				for i in range(len(kin_markers)):
					with self.env:
						self.env.Remove(kin_markers[i])
				for j in range(len(actual_points)):
					#new markers
					T = np.eye(4)
					container_center = np.array([actual_points[j][0], actual_points[j][1], actual_points[j][2]])
					T[:3, 3] = container_center
					#R = []
					R = deepcopy(self.__rotation)
					T[:3,:3] = R[:3,:3]
					kin_marker = create_box(T, color = [0, 0, 0.6])
					with self.env:
						if self.env.CheckCollision(kin_marker):
							self.env.Remove(kin_marker)	
						else:
							newkin_markers.append(kin_marker)

							
				self.__kin_markers = newkin_markers
		#else:
			#print ' no marker loaded '
		
		return True
	
	def load_image (self, frame):
		self.__frame = deepcopy(frame)
		#print' loading image '
		font = cv2.FONT_HERSHEY_SIMPLEX
		for i in self.__markers:
			x = int(self.__markers[i][0][1][0])
			y = int(self.__markers[i][0][1][1])
			#print 'drawing',(x,y)
			#print 'number ' , i ,'position ', self.__markers[i][0][0]
			if x !=0 and y!=0:
				cv2.putText(self.__frame, str(i), (x,y), font, 1.0, (0, 255, 0), 3)
				cv2.circle(self.__frame,(x,y), 1, (0,0,255), -1)
				#cv2.imshow('trackingimage', self.__frame)
				#cv2.waitKey(30)
		
	
	def get_image(self):
		return deepcopy(self.__frame)
	
	
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
		
		if self.__start_track == True:
			#with lock:
			#print'Starting marker tracking'
			if self.__reset == True:
				self.__tracking_marker = []
				self.__tracking_marker2 = []
				self.__reset = False
				print 'Reset tracking '
			else:
				if len(self.__tracking_marker2) != 0:
					# track if there is a postion
					for i in range(len(self.__tracking_marker2)):
						tracking_markers2 = deepcopy(self.__tracking_marker2[i])
						try:
							if self.__markers[tracking_markers2][0][0][0] is not None:
								#print ('Tracking of ', self.__tracking_marker2)
								#add only if different from last point
								length = 0
								current_position = deepcopy(self.__markers[tracking_markers2][0][0])
								diffix = current_position[0] - self.__markers2[tracking_markers2][len(self.__markers2[tracking_markers2])-1][0][0]
								diffiy = current_position[1] - self.__markers2[tracking_markers2][len(self.__markers2[tracking_markers2])-1][0][1]
								diffiz = current_position[2] - self.__markers2[tracking_markers2][len(self.__markers2[tracking_markers2])-1][0][2]
								length = np.sqrt( np.power(diffix,2) + np.power(diffiy,2) +np.power(diffiz,2))
								if length >=0.01 or length <=-0.01:
									print 'tracking list updated '
									marker_to_add = deepcopy(self.__markers[tracking_markers2][0])
									self.__markers2[tracking_markers2].append(marker_to_add)
									print len(self.__markers2[tracking_markers2])
						except KeyError:
							new_p = deepcopy(self.__markers[tracking_markers2][0][0])
							new_pl = deepcopy(self.__markers[tracking_markers2][0][1])
							self.__markers2[tracking_markers2] = [[new_p,new_pl]]
							
		return deepcopy(True)					
						#else:
							#print 'length too small', length
					#else:
						#print 'nothing to track'
				#else:
					#print ' no marker selected '
		#else:
			#print ' not tracking'
			
	def Move_along_path(self, path, lock):
		print'moving robot'
		
		# Move to the grasping pose
		def plan_to_joint_values(qgoal):
			# Create the planner
			planner = orpy.RaveCreatePlanner(env,'birrt') # Using bidirectional RRT
			params = orpy.Planner.PlannerParameters()
			params.SetRobotActiveJoints(robot)
			params.SetGoalConfig(qgoal)
			params.SetExtraParameters('<_postprocessing planner="ParabolicSmoother"><_nmaxiterations>40</_nmaxiterations></_postprocessing>')
			planner.InitPlan(robot, params)
			# Plan a trajectory
			traj = orpy.RaveCreateTrajectory(env, '')
			planner.PlanPath(traj)
			return traj
		
		with lock:
			env = self.env
			robot = env.GetRobot('robot')
			manipulator = robot.SetActiveManipulator('gripper')
			robot.SetActiveDOFs(manipulator.GetArmIndices())
			
			R = []
			R = self.__rotation
			Tgrasp =np.eye(4)
			Ry = orpy.matrixFromAxisAngle(np.array((0,1,0)),np.pi)
			R_grasp = np.dot(R,Ry)
			z_direction = np.array([0,0,0.02])
			z_shift = np.dot(R[:3,:3],z_direction)
			
			Tgrasp[:3,:3] = R_grasp[:3,:3]
			Tgrasp[:3,3] = path[0]+z_shift
			
			#move to first point
			iktype=orpy.IkParameterizationType.Transform6D
			ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=iktype)
			if not ikmodel.load():
				print 'Generating IKFast {0}. It will take few minutes...'.format(iktype.name)
				ikmodel.autogenerate()
				print 'IKFast {0} has been successfully generated'.format(iktype.name)
			
			qgrasp = None
			ikparam = orpy.IkParameterization(Tgrasp, iktype)
			solutions = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
			if len(solutions) == 0:
				raise Exception('Failed to find a valid IK for marker position')
			qgrasp = solutions[0]
			first_traj = plan_to_joint_values(qgrasp)
			controller = robot.GetController()
			controller.SetPath(first_traj)
			robot.WaitForController(0)
			#move alone the path
			traj = orpy.RaveCreateTrajectory(env,'')
			spec = orpy.IkParameterization.GetConfigurationSpecificationFromType(orpy.IkParameterizationType.Transform6D,'linear')
			traj.Init(spec)
			Tgrasp =np.eye(4)
			number_fail = 0
			for i in range(len(path)):
				Tgrasp[:3,:3] = R_grasp[:3,:3]
				Tgrasp[:3,3] = path[i]+z_shift
				ikparam = orpy.IkParameterization(Tgrasp, iktype)
				solutions = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
				if len(solutions) == 0:
					raise Exception('Failed to find a valid IK for marker position')
					number_fail +=1
				traj.Insert(traj.GetNumWaypoints(),orpy.poseFromMatrix(Tgrasp))
			if number_fail ==0:
				orpy.planningutils.RetimeAffineTrajectory(traj,maxvelocities=0.5*np.ones(7),maxaccelerations=5*np.ones(7))
				with env:
					planner = orpy.RaveCreatePlanner(env,'workspacetrajectorytracker')
					params = orpy.Planner.PlannerParameters()
					params.SetRobotActiveJoints(robot)
					params.SetExtraParameters('<workspacetrajectory>%s</workspacetrajectory>'%traj.serialize(0))
					planner.InitPlan(robot,params)
					outputtraj = orpy.RaveCreateTrajectory(env,'')
					success=planner.PlanPath(outputtraj)
				if success == True:
					print ' path successful plan. '
					robot.GetController().SetPath(outputtraj)
					robot.WaitForController(0)
				else:
					print ' not successful path. '
					for i in range(len(path)):
						Tgrasp[:3,:3] = R_grasp[:3,:3]
						Tgrasp[:3,3] = path[i]+z_shift
						ikparam = orpy.IkParameterization(Tgrasp, iktype)
						solutions = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
						qgrasp = solutions[0]
						first_traj = plan_to_joint_values(qgrasp)
						controller.SetPath(first_traj)
						robot.WaitForController(0)
					#embed()
			else:
				print' fail to find solutions for all points '
			
			#clear
			raw_input(' press any key to clear and continue. ')
			traj = plan_to_joint_values(np.zeros(6))
			controller = robot.GetController()
			controller.SetPath(traj)
			robot.WaitForController(0)
			
			self.__markers = {}
			#self.__environment = False
			self.__tracking_marker = []
			print 'data clear '
		return True
		
	def load_marker_path (self, size, allsize, all_position):
		new_all_position = deepcopy(all_position)
		for i in size:
			j = all_size[i]
			for k in range(j):
				self.__cppmarkers[i].append(new_all_position[k])
			for l in range(j):
				new_all_position.remove(new_all_postion[l])
				self.__markersloaded = True
							
				
	def retrieve_marker_path (self,lock):
		with lock:
			tracking_marker = raw_input('choose which marker path ')
			try:
				tracking_marker = int(tracking_marker)
			except ValueError:
				print' not a number int '
			print ('retrieving path of ',tracking_marker)
			self.__tracking_list = []
			'''
			for i in range(len(self.__markers2[tracking_marker])):
				self.__tracking_list.append(self.__markers2[tracking_marker][i][0])
			self.__tracking_marker = []
			return deepcopy(self.__tracking_list)
			'''
			for i in range(len(self.__cppmarkers[tracking_marker])):
				self.__tracking_list.append(self.__cppmarkers[tracking_marker][i])
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
		self.__environment = False
		self.__reset = False
		self.__no_change = False
		self.__tracking_marker = []
		self.__tracking_marker2 = []
		self.__rotation = np.eye(4)
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
	'''
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
	'''
	
	