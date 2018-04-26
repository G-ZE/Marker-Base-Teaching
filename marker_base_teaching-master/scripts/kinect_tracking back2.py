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
				



#not used
def imageandpoints_callback(msg):

	global get
	
	print 'calling points '
	newpoints = []
	actual_points = []
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
	actual_points = deepcopy(newpoints)
	dm.load_markers_position(actual_points,pixel_points)
	
	frame = bridge.imgmsg_to_cv2(msg.image, desired_encoding="passthrough")
	BGR = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
	
	dm.load_image(BGR)
	if len(actual_points) !=0:
		print 'got data '
		get = True

def image_callback(data):
	global tracker_sub_once
	global get_image
	image_buffer = 0
	frame = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
	#dm.load_image(frame)
	get_image = True
	print 'got image '
	font = cv2.FONT_HERSHEY_SIMPLEX
	c_markers = dm.retrieve_markers_data()
	for i in c_markers:
		x = int(c_markers[i][0][1][0])
		y = int(c_markers[i][0][1][1])
		cv2.putText(frame, str(i), (x,y), font, 1.0, (0, 255, 0), 2)
		cv2.circle(frame,(x,y), 1, (0,0,255), -1)
	cv2.imshow('tracking_image', frame)
	cv2.waitKey(30)
	if cv2.waitKey(30) & 0xFF == ord('q'):
		tracker_sub_once.unregister()
		cv2.destroyAllWindows()
#not used
def create_box(T, color = [0, 0.6, 0]):
	box = orpy.RaveCreateKinBody(env, '')
	box.SetName('box')
	box.InitFromBoxes(np.array([[0,0,0,0.01,0.01,0.005]]), True)    
	g = box.GetLinks()[0].GetGeometries()[0]
	g.SetAmbientColor(color)
	g.SetDiffuseColor(color)
	box.SetTransform(T)
	env.Add(box,True)
	return box
	
	
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

def background():
	timeset = 10.0
	while True:
		#geting points
		print ' retieving markers data '
		get = False
		sub_once = rospy.Subscriber("kinect_openrave_marker_points", markers, imageandpoints_callback, queue_size =1)
		begin = rospy.get_rostime().secs
		currenttime =rospy.get_rostime().secs
		while ((currenttime-begin) < timeset) and get != True:
			currenttime =rospy.get_rostime().secs
		sub_once.unregister()
		frame = dm.get_image()
		cv2.imshow('tracking_image', frame)
		cv2.waitKey(30)
		print ' shutdown subscriber '
	
	

if "__main__" == __name__:
	try:
		print "Kinect tracking Start"
		rospy.init_node("kinect_tracking", anonymous=True)
		dm = Datamanagement.Markerdata()
		
		
		
		#global newpoints
		#global sub_once
		#sub_once = rospy.Subscriber("kinect_openrave_marker_points", markers, points_callback, queue_size =1)
		
		global get
		get= False
		
		threading1 = threading.Thread(target=background)
		threading1.daemon = True
		threading1.start()
		lock = threading.Lock()
		
		bridge = CvBridge()
		kin_markers = []
		actual_points = []
		pixel_points = []
		timeset = 10.0
	
		
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
			'''
			# loading environment
			#Imsg = rospy.wait_for_message("/camera/rgb/image_rect_color", Image)
			#frame = bridge.imgmsg_to_cv2(Imsg, desired_encoding="passthrough")
			#dm.load_image(frame)
			
			'''
			m_data = rospy.wait_for_message("/camera/rgb/image_rect_color", Image)
			frame = bridge.imgmsg_to_cv2(m_data, desired_encoding="passthrough")
			#dm.load_image(frame)
			data = dm.retrieve_markers_data()
			font = cv2.FONT_HERSHEY_SIMPLEX
			embed()
			cv2.imshow('tracker_image', frame) 
			
			for i in data:
				x = int(data[i][0][1][0])
				y = int(data[i][0][1][1])
				cv2.putText(frame, str(i), (x,y), font, 1.0, (0, 255, 0), 2)
			
			'''
			#tracker_sub_once = rospy.Subscriber("/camera/rgb/image_rect_color", Image, image_callback)
			
			
			
			
			#dm.load_environment(lock)
			'''
			#creating markers
			newkin_markers = []
			if len(actual_points) > 0 and len(kin_markers) == 0:
				# first time detected
				print " creating markers"
				for i in range(len(actual_points)):
					T = np.eye(4)
					
					container_center = np.array([actual_points[i][0], actual_points[i][1], actual_points[i][2]])
					T[:3, 3] = container_center
					R = []
					R = dm.get_rotation()
					T[:3,:3] = R[:3,:3]
					kin_marker = create_box(T, color = [0, 0, 0.6])
					kin_markers.append(kin_marker)
					
			elif len(actual_points)> 0 and len(kin_markers) !=0:
				# second and more detected
				# creating new markers and remove markers that are not there
				
				# old points in actualpoints
				print " keeping existing markers"
				for i in range(len(kin_markers)):
					for j in range(len(actual_points)):
						old_marker = kin_markers[i].GetTransform()[:3,3]
						dist = np.sqrt(np.square(old_marker[0]) + np.square(old_marker[1]) + np.square(old_marker[2]))
						dist2 = np.sqrt(np.square(actual_points[j][0]) + np.square(actual_points[j][1]) + np.square(actual_points[j][2]))
						if (dist-dist2) == 0.01:
							newkin_markers.append(kin_markers[i])
							kin_markers.Remove(kin_markers[i])
							actual_points.Remove(actual_points[j])
				
				print " creating new markers and remove non markers"
				for i in range(len(kin_markers)):
					env.Remove(kin_markers[i])
				for j in range(len(actual_points)):
					#new markers
					T = np.eye(4)
					container_center = np.array([actual_points[j][0], actual_points[j][1], actual_points[j][2]])
					T[:3, 3] = container_center
					R = []
					R = dm.get_rotation()
					T[:3,:3] = R[:3,:3]
					kin_marker = create_box(T, color = [0, 0, 0.6])
					if env.CheckCollision(kin_marker):
						env.Remove(kin_marker)
							
					else:
						newkin_markers.append(kin_marker)

						
				kin_markers = newkin_markers
			
			'''
			#dm.marker_tracking()
			#dm.marker_tracking2(lock)
			#embed()
			path =[]
			print 'press d to retrieve path'
			entered = raw_input()
			if raw_input == 'd':
				print ' got input '
				path = dm.retrieve_marker_path(lock)
				embed()
			#if cv2.waitKey(30) & 0xFF == ord('d'):
				#path = dm.retrieve_marker_path()
				#embed()
			else:
				print' not retrieving path '
				
			if len(path) !=0:
				print'moving robot'
				
				dm.Move_along_path(path,lock)
				'''
				robot = env.GetRobot('robot')
				embed()
				manipulator = robot.SetActiveManipulator('gripper')
				robot.SetActiveDOFs(manipulator.GetArmIndices())
				
				R = []
				R = dm.get_rotation()
				Tgrasp =np.eye(4)
				Ry = orpy.matrixFromAxisAngle(np.array((0,1,0)),np.pi)
				R_grasp = np.dot(R,Ry)
				Tgrasp[:3,:3] = R_grasp[:3,:3]
				Tgrasp[:3,3] = path[0]
				
				#move to first point
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
				for i in len(path):
					Tgrasp[:3,:3] = R_grasp[:3,:3]
					Tgrasp[:3,3] = path[i]
					ikparam = orpy.IkParameterization(Tgrasp, iktype)
					solutions = manipulator.FindIKSolutions(ikparam, orpy.IkFilterOptions.CheckEnvCollisions)
					if len(solutions) == 0:
						raise Exception('Failed to find a valid IK for marker position')
					traj.Insert(traj.GetNumWaypoints(),orpy.poseFromMatrix(Tgrasp))
				
				orpy.planningutils.RetimeAffineTrajectory(traj,maxvelocities=np.ones(7),maxaccelerations=5*np.ones(7))
				with env:
					planner = orpy.RaveCreatePlanner(env,'workspacetrajectorytracker')
					params = orpy.Planner.PlannerParameters()
					params.SetRobotActiveJoints(robot)
					params.SetExtraParameters('<workspacetrajectory>%s</workspacetrajectory>'%traj.serialize(0))
					planner.InitPlan(robot,params)
        
					outputtraj = orpy.RaveCreateTrajectory(env,'')
					success=planner.PlanPath(outputtraj)
				
				robot.GetController().SetPath(outputtraj)
				robot.WaitForController(0)
				
				#clear
				print ' press any key to clear and continue. '
				cv2.waitKey(0)
				traj = plan_to_joint_values(np.zeros(6))
				controller = robot.GetController()
				controller.SetPath(traj)
				robot.WaitForController(0)
				dm.clear_data()
				'''
			
	except rospy.ROSInterruptException:
		pass
