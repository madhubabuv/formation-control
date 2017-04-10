#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class MarkerArrayDetection:
	
	def __init__(self,marker_id,marker_size):
		print "MarkerDetection Started"
		self.DistCoefficents=np.array([-0.526629354780687, 0.274357114262035, 0.0211426202132638, -0.0063942451330052, 0])
		self.CameraMatrix=np.array([[569.883158064802, 0, 331.403348466206], [0, 568.007065238522, 135.879365106014], [0, 0, 1]])
		self.tvecs,self.rvecs=[],[]
		self.ids,self.corners=0,0
		self.pose=[]		
		self.ImageSubscriber=None
		self.status=False		
		self.MarkerPose=[]
		self.marker_id=marker_id
		self.marker_size=marker_size
	def FindPose(self,corners):
		#print i
		self.rvecs,self.tvecs=cv2.aruco.estimatePoseSingleMarkers(corners,self.marker_size,self.CameraMatrix,self.DistCoefficents)
		R, J = cv2.Rodrigues(np.array(self.rvecs)) 
		self.MarkerPose=np.dot(-R.T,self.tvecs[0].T)
		corners=corners.ravel()
		self.marker_center=[float(corners[0]+corners[2]+corners[4]+corners[6])/4,float(corners[1]+corners[3]+corners[5]+corners[7])/4]
		self.tvecs=self.tvecs.ravel()
		#self.MarkerPose=[self.MarkerPose[2],self.tvecs[0],self.tvecs[1],self.tvecs[2]]
		#print self.MarkerPose[2],self.tvecs[0],self.tvecs[1]

	def DetectMarker(self,frame):
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		#self.marker_detect=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
		self.marker_detect=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
		self.corners,self.ids,reject=cv2.aruco.detectMarkers(gray,self.marker_detect)
		if self.ids!=None:
			for i in range(len(self.ids)):
				if self.ids[i]==self.marker_id:
					self.FindPose(self.corners[i])
					return self.tvecs,self.rvecs
				else:
					return None,None
		else:
			return None,None
	def RosImageCallBack(self,ros_image):
		bridge = CvBridge()
		self.marker_now=rospy.get_rostime().to_sec()
		try:         
			frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")  # converting the sensor_msgs/image type to IplImage	
			pose,orientation=self.DetectMarker(frame)
			if pose!=None:
				cv2.circle(frame,(640/2,360/2),5,(255,255,0),-1)
				cv2.circle(frame,(int(self.marker_center[0]),int(self.marker_center[1])),5,(255,0,0),-1)
				cv2.line(frame,(640/2,360/2),(int(self.marker_center[0]),int(self.marker_center[1])),(255,0,255))
			cv2.imshow("image",frame)
			cv2.waitKey(1)
			return pose,orientation
		except CvBridgeError, e:
			print e	
'''if __name__=='__main__':
	rospy.init_node("MarkerArrayDetection",anonymous=True)
	marker_obj=MarkerArrayDetection()
	rate=rospy.Rate(5)
	print "Hello"
	while not rospy.is_shutdown():
		ros_image = rospy.wait_for_message("/ardrone/front/image_raw",Image)
		marker_obj.RosImageCallBack(ros_image)
		rate.sleep()'''


