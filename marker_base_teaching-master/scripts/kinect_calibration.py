#!/usr/bin/env python

# Perception
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from copy import deepcopy
import openravepy as orpy
from IPython import embed
import yaml

# Messages
from sensor_msgs.msg import Image

bridge = CvBridge()

'''
H_minvalue
S_minvalue
V_minvalue
H_maxvalue
S_maxvalue
V_maxvalue
minarea
maxarea 
minarea_ratio
maxarea_ratio 
minlength_ratio 
maxlength_ratio
'''

H_minvalue = 120
S_minvalue = 40
V_minvalue = 0

H_maxvalue = 168
S_maxvalue = 255
V_maxvalue = 255

H_minvalue_g = 30
S_minvalue_g = 20
V_minvalue_g = 0

H_maxvalue_g = 90
S_maxvalue_g = 255
V_maxvalue_g = 255

minarea = 100
maxarea = 1100
minarea_ratio = 3
maxarea_ratio = 9
minlength_ratio = 1
maxlength_ratio = 4

def image_callback(data):
	global sub_once
	frame = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
	#BGR = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
	#dst = cv2.fastNlMeansDenoisingColored(frame,None,5,5,7,21)
	blur = cv2.medianBlur(frame,3)
	#HSV2 = cv2.cvtColor(dst,cv2.COLOR_BGR2HSV)
	HSV = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
	#HSV2 = cv2.cvtColor(BGR,cv2.COLOR_BGR2HSV)
	
	minHSV = np.array([H_minvalue,S_minvalue,V_minvalue])
	maxHSV = np.array([H_maxvalue,S_maxvalue,V_maxvalue])
	
	g_minHSV = np.array([H_minvalue_g,S_minvalue_g,V_minvalue_g])
	g_maxHSV = np.array([H_maxvalue_g,S_maxvalue_g,V_maxvalue_g])
	
	#maskHSV2 = cv2.inRange(HSV2, minHSV, maxHSV)
	maskHSV = cv2.inRange(HSV, minHSV, maxHSV)
	maskHSV_g = cv2.inRange(HSV, g_minHSV, g_maxHSV)
	#maskHSV2 = cv2.inRange(HSV2, minHSV, maxHSV)
	emarks = cv2.erode(maskHSV, None, iterations=1)
	dmarks = cv2.dilate(emarks, None, iterations=1)
	emarks_g = cv2.erode(maskHSV_g, None, iterations=1)
	dmarks_g = cv2.dilate(emarks_g, None, iterations=1)
	#emarks2 = cv2.erode(maskHSV2, None, iterations=1)
	#dmarks2 = cv2.dilate(emarks2, None, iterations=1)
	
	#im2, contours, hierarchy = cv2.findContours(maskHSV,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	im2, contours, hierarchy = cv2.findContours(dmarks,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	im2_g, contours_g, hierarchy_g = cv2.findContours(dmarks_g,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#im2, contours, hierarchy = cv2.findContours(maskHSV2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	
			
	for i in range(len(contours)):
		carea = cv2.contourArea(contours[i])
		if carea > minarea and maxarea > carea:
			
			
			#print "i is ", i , " area ", carea
			for j in range(len(hierarchy[0])):
				
				if hierarchy[0][j][3] == i and hierarchy[0][j][2]==-1:
					childarea = cv2.contourArea(contours[j])
					area_ratio = carea/childarea
					clength = cv2.arcLength(contours[i],True)
					childlength = cv2.arcLength(contours[j],True)
					length_ratio = clength/childlength
					
					#print "j is ", j ," area_ratio ", area_ratio, "length_ratio ", length_ratio
					pause = False
					if area_ratio >= minarea_ratio and maxarea_ratio >= area_ratio and length_ratio >= minlength_ratio and maxlength_ratio >= length_ratio:
						
						M = cv2.moments(contours[i])
						cx = M['m10']/M['m00']
						cy = M['m01']/M['m00']
						M2 = cv2.moments(contours[j])
						cx2 = M2['m10']/M2['m00']
						cy2 = M2['m01']/M2['m00']
						deta_x = cx-cx2
						deta_y = cy-cy2
						pause = True
						
						ellipse = cv2.fitEllipse(contours[i])
						ma = ellipse[1][1]
						MA = ellipse[1][0]
						ellipse_area = np.pi*MA*ma/4
						other_area_ratio = ellipse_area/carea
						
						e_cx = ellipse[0][0]
						e_cy = ellipse[0][1]
						e_deta_x = cx-e_cx
						e_deta_y = cy-e_cy
						
						cv2.drawContours(frame, contours[i], -1, (0,255,0), 1)
						cv2.drawContours(frame, contours[j], -1, (0,255,0), 1)
						for k in range(len(contours_g)):
							carea = cv2.contourArea(contours_g[k])
							if 1100 > carea:
								cv2.drawContours(maskHSV_g, contours_g[k], -1, (0,0,255), 1)
								M_g = cv2.moments(contours_g[k])
								cx_g = M_g['m10']/M_g['m00']
								cy_g = M_g['m01']/M_g['m00']
								deta_x_g = cx-cx_g
								deta_y_g = cy-cy_g
								
								if deta_x_g <= 1 and deta_x_g >= -1 and deta_y_g <= 1 and deta_y_g >= -1:
									print "deta_x_g ", deta_x_g, "deta_y_g ", deta_y_g	
									cv2.drawContours(frame, contours[i], -1, (0,0,255), 1)
									cv2.drawContours(frame, contours[j], -1, (255,0,0), 1)
							
						#print "deta_x ", deta_x, " e_x ", e_deta_x, "deta_y ", deta_y, " e_y ", e_deta_y
						#print " ellipse and contour area ratio ", other_area_ratio
						'''
						if deta_x <= 1 and deta_x >= -1 and deta_y <= 1 and deta_y >= -1:
							if other_area_ratio >=1 and other_area_ratio <=1.5:
								cv2.ellipse(frame,ellipse,(0,255,150),2)
								cv2.drawContours(frame, contours[i], -1, (0,0,255), 1)
								cv2.drawContours(frame, contours[j], -1, (255,0,0), 1)
							#embed()
						'''
						'''
						if pause == True:
							from IPython import embed
							embed()
						'''
			
	
	#cv2.imshow('MASKHSV2', maskHSV2)
	cv2.imshow('dilate', dmarks)
	#cv2.imshow('dilate', dmarks2)		
	cv2.imshow('image', frame)
	#cv2.imshow('image2', frame)
	cv2.imshow('MASKHSV', maskHSV)
	cv2.imshow('MASKHSV_g', maskHSV_g)
	#cv2.imshow('image3', dst)
	#cv2.imshow('MASKHSV2', maskHSV2)
	cv2.waitKey(30)
	
	cv2.createTrackbar('mH','image',H_minvalue,180,mH_callback)
	cv2.createTrackbar('MH','image',H_maxvalue,180,MH_callback)
	cv2.createTrackbar('mS','image',S_minvalue,255,mS_callback)
	cv2.createTrackbar('mV','image',V_minvalue,255,mV_callback)
	#cv2.createTrackbar('mA','image',minarea,500,mA_callback)
	#cv2.createTrackbar('MA','image',maxarea,3000,MA_callback)
	#cv2.createTrackbar('mAratio','image2',minarea_ratio,10,mAR_callback)
	#cv2.createTrackbar('MAratio','image2',maxarea_ratio,10,MAR_callback)
	#cv2.createTrackbar('mLratio','image2',minlength_ratio,4,mLR_callback)
	#cv2.createTrackbar('MLratio','image2',maxlength_ratio,4,MLR_callback)
	
	'''
	#cv2.namedWindow('image')
	
	cv2.getTrackbarPos('mH','image')
	cv2.getTrackbarPos('mS','image')
	cv2.getTrackbarPos('mV','image')
	cv2.getTrackbarPos('MH','image')
	cv2.getTrackbarPos('MS','image')
	cv2.getTrackbarPos('MV','image')
	cv2.getTrackbarPos('mA','image')
	cv2.getTrackbarPos('MA','image')
	cv2.getTrackbarPos('mAratio','image')
	cv2.getTrackbarPos('MAratio','image')
	cv2.getTrackbarPos('mLratio','image')
	cv2.getTrackbarPos('MLratio','image')
	'''
	
	
	
	if cv2.waitKey(30) & 0xFF == ord('q'):
		sub_once.unregister()
		cv2.destroyAllWindows()
		
		
def listener():
	rospy.init_node("kinect_calibration", anonymous=True)
	
	
	global sub_once
	sub_once = rospy.Subscriber("/camera/rgb/image_rect_color", Image, image_callback)
	
	rospy.spin()
	
	

def mH_callback(value):
	global H_minvalue
	H_minvalue = deepcopy(value)
	
def MH_callback(value):
	global H_maxvalue
	H_maxvalue = deepcopy(value)
	
def mS_callback(value):
	global S_minvalue
	S_minvalue = deepcopy(value)
	
def MS_callback(value):
	global S_maxvalue
	S_maxvalue = deepcopy(value)
	
def mV_callback(value):
	global V_minvalue
	V_minvalue = deepcopy(value)
	
def MV_callback(value):
	global V_maxvalue
	V_maxvalue = deepcopy(value)
	
def mA_callback(value):
	global minarea
	minarea = deepcopy(value)
	
def MA_callback(value):
	global maxarea
	maxarea = deepcopy(value)
	
def mAR_callback(value):
	global minarea_ratio
	minarea_ratio = deepcopy(value)
	
def MAR_callback(value):
	global maxarea_ratio
	maxarea_ratio = deepcopy(value)
	
def mLR_callback(value):
	global minlength_ratio
	minlength_ratio = deepcopy(value)
	
def MLR_callback(value):
	global maxlength_ratio
	maxlength_ratio = deepcopy(value)
	
	
def singleimage():
	
	img = cv2.imread('/home/goh/Desktop/camera/capture_screenshot_10.10.2017-6.png',1)
	BGR = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
	blur = cv2.medianBlur(BGR,3)
	HSV = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
	
	
	cv2.namedWindow('image')
	cv2.createTrackbar('mH','image',0,180,mH_callback)
	'''
		cv2.createTrackbar('mH','image',0,180,mH_callback)
		cv2.createTrackbar('mS','image',0,255,mS_callback)
		cv2.createTrackbar('mV','image',0,255,mV_callback)
		cv2.createTrackbar('MH','image',0,180,MH_callback)
		cv2.createTrackbar('MS','image',0,255,MS_callback)
		cv2.createTrackbar('MV','image',0,255,MV_callback)
		cv2.createTrackbar('mA','image',0,500,mA_callback)
		cv2.createTrackbar('MA','image',0,3000,MA_callback)
		cv2.createTrackbar('mAratio','image',0,10,mAR_callback)
		cv2.createTrackbar('MAratio','image',0,10,MAR_callback)
		cv2.createTrackbar('mLratio','image',0,4,mLR_callback)
		cv2.createTrackbar('MLratio','image',0,4,MLR_callback)
	'''
	while True:
		minHSV = np.array([H_minvalue,S_minvalue,V_minvalue])
		maxHSV = np.array([H_maxvalue,S_maxvalue,V_maxvalue])
		maskHSV = cv2.inRange(HSV, minHSV, maxHSV)
		emarks = cv2.erode(maskHSV, None, iterations=1)
		dmarks = cv2.dilate(emarks, None, iterations=1)
		im2, contours, hierarchy = cv2.findContours(dmarks,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for i in range(len(contours)):
			carea = cv2.contourArea(contours[i])
			if carea > minarea and carea < maxarea:
					
				cv2.drawContours(img, contours, i, (0,255,0), 1)
				print "i is ", i , " area ", carea
					
				for j in range(len(hierarchy[0])):
						
					if hierarchy[0][j][3] == i and hierarchy[0][j][2]==-1:
							
						childarea = cv2.contourArea(contours[j])
						area_ratio = carea/childarea
						clength = cv2.arcLength(contours[i],True)
						childlength = cv2.arcLength(contours[j],True)
						length_ratio = clength/childlength
						print "j is ", j ," area_ratio ", area_ratio, "length_ratio ", length_ratio
							
						if area_ratio >= minarea_ratio and area_ratio <= maxarea_ratio and length_ratio>= minlength_ratio and length_ratio <= maxlength_ratio:
							cv2.drawContours(img, contours[i], -1, (0,0,255), 1)
							cv2.drawContours(img, contours[j], -1, (255,0,0), 1)
							M = cv2.moments(contours[i])
							cx = int(M['m10']/M['m00'])
							cy = int(M['m01']/M['m00'])
							M2 = cv2.moments(contours[j])
							cx2 = int(M2['m10']/M2['m00'])
							cy2 = int(M2['m01']/M2['m00'])
							deta_x = cx-cx2
							deta_y = cy-cy2
							print "deta_x ", deta_x, "deta_y ", deta_y
			
		
		cv2.imshow('image',img)
		cv2.imshow('image2',img)
		cv2.imshow('MASKHSV', maskHSV)
		cv2.imshow('dilate', dmarks)
		cv2.waitKey(30)
		
		if cv2.waitKey(30) & 0xFF == ord('q'):
			cv2.destroyAllWindows()
			from IPython import embed
			#embed()
		
	

if "__main__" == __name__:
	try:
		print "Kinect Calibration Start"
		# Set calibration pattern
		#rospy.init_node("kinect_calibration", anonymous=True)
		
		#tracking_data = {}
		#tracking_data[0] = 1
		#tracking_data[1] = 2
		#with open("/home/goh/catkin_ws/src/marker_base_teaching-master/kinect_retrieve_saves/tracking_Data.yml","w") as outfile:
			#yaml.dump(tracking_data, outfile, default_flow_style=False)
		
		
		#env = orpy.Environment()

		#env.Load('/home/goh/catkin_ws/src/osr_course_pkgs/osr_openrave/worlds/just_robot.xml')
		#env.SetDefaultViewer()
		
		#msg = rospy.wait_for_message("/camera/rgb/image_rect_color", Image)
	
		#frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
		#cv2.imshow('image', frame)
		
		
		
		
		listener()
	
		#singleimage()
		
		
	except rospy.ROSInterruptException:
		pass
