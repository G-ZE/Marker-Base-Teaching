#!/usr/bin/env python

# Perception
import rospy
import numpy as np
import openravepy as orpy
from copy import deepcopy
import time
# Messages
from marker_base_teaching.msg import markers

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
				




def points_callback(msg):
	global sub_once
	global newpoints
	if len(newpoints) > 0:
		newpoints = []
		for i in range(len(msg.arraypoints)):
			msgpoints =[msg.arraypoints[i]]
			p = msgpoints[0]
			point = np.array([p.x,p.y,p.z])
			newpoints.append(point)
	else:
		for i in range(len(msg.arraypoints)):
			msgpoints =[msg.arraypoints[i]]
			p = msgpoints[0]
			point = np.array([p.x,p.y,p.z])
			newpoints.append(point)
	sub_once.unregister()
	
	
	
	
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

if "__main__" == __name__:
	try:
		print "Kinect openrave Start"
		rospy.init_node("kinect_openrave", anonymous=True)
		#global newpoints
		newpoints = []
		#global sub_once
		#sub_once = rospy.Subscriber("kinect_openrave_marker_points", markers, points_callback, queue_size =1)
		
		
		
		env = orpy.Environment()
		env.SetDefaultViewer()
		

		count = 0
		kin_markers = []
		stop = 0
		actual_points = []
		check_points = []
		timeset = 5.0
		
		while True:
			#geting points
			while True:
				print " getting points"
				msg = rospy.wait_for_message("kinect_openrave_marker_points", markers)
				
				newpoints = []
				for i in range(len(msg.arraypoints)):
					msgpoints =[msg.arraypoints[i]]
					p = msgpoints[0]
					point = np.array([p.x,p.y,p.z])
					newpoints.append(point)
				
				actual_points = deepcopy(newpoints)
				
				
				'''
				if len(newpoints) > 0:
					newpoints = []
					for i in range(len(msg.arraypoints)):
						msgpoints =[msg.arraypoints[i]]
						p = msgpoints[0]
						point = np.array([p.x,p.y,p.z])
						newpoints.append(point)
				else:
					for i in range(len(msg.arraypoints)):
						msgpoints =[msg.arraypoints[i]]
						p = msgpoints[0]
						point = np.array([p.x,p.y,p.z])
						newpoints.append(point)
			
				'''
				'''
				print count
				if count == 0 and len(newpoints) > 0:
					print "keeping first points"
					check_points = deepcopy(newpoints)
					count +=1
					
				elif count == 1 and len(newpoints) >0:
					print "checking points"
					actual_points = []
					for i in range(len(check_points)):
						for j in range(len(newpoints)):
							marker_vector = check_points[i] - newpoints[j]
						
							dist = np.sqrt(np.square(marker_vector[0]) + np.square(marker_vector[1]) + np.square(marker_vector[2]))
						
							if dist < 0.007:
								point_to_add = deepcopy(newpoints[j])
								actual_points.append(point_to_add)
							
					count = 0
				'''
				if len(actual_points) > 0:
					break
			
			
			#creating markers
			newkin_markers = []
			if len(actual_points) > 0 and len(kin_markers) == 0:
				# first time detected
				print " creating markers"
				for i in range(len(actual_points)):
					T = np.eye(4)
					container_center = np.array([actual_points[i][0], actual_points[i][1], actual_points[i][2]])
					T[:3, 3] = container_center
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
					kin_marker = create_box(T, color = [0, 0, 0.6])
					if env.CheckCollision(kin_marker):
						env.Remove(kin_marker)
							
					else:
						newkin_markers.append(kin_marker)

						
				kin_markers = newkin_markers
			'''	
			elif len(actual_points) ==0 and len(kin_markers)!=0:
				# for no markers
				for i in range(len(kin_markers)):
					env.Remove(kin_markers[i])
						
				kin_markers = []
			'''
	except rospy.ROSInterruptException:
		pass
