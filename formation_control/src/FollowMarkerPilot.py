#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Vector3
from std_msgs.msg import Empty
from PIDAutoTuning import PIDAutoTuning
from nav_msgs.msg import Odometry
from MarkerArrayDetection import MarkerArrayDetection
from sensor_msgs.msg import Image

#use marker Id 101

class AutoPilot:

	def __init__(self,target,alt_target,marker_id,marker_size):
		
		self.pitch_pid=PIDAutoTuning(0.025,0.007,0.5,0.3,0.0,False) #0.025 0.007 with error limit 0.1
		self.roll_pid=PIDAutoTuning(0.025,0.007,0.5,0.3,0.0,False)
		self.alt_pid=PIDAutoTuning(0.03,0.005,0.6,0.1,0.01,False)
		self.yaw_pid=PIDAutoTuning(0.005,0.007,0.3,0.1,0.0005,True)
		self.marker=MarkerArrayDetection(marker_id,marker_size)
		self.buffer_len=5
		self.prev_yaw=0
		self.state_buffer=[]
		self.target=target
		self.alt_target=alt_target

		self.now=rospy.get_rostime().to_sec()
		self.pubTakeoff=rospy.Publisher("/ardrone/takeoff",Empty,queue_size=1)
		self.pubLand=rospy.Publisher("/ardrone/land",Empty,queue_size=1)
		self.pubcmd=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
		self.pub_pitch=rospy.Publisher("/ardrone/pitch_pid",Vector3,queue_size=1)
		self.pub_roll=rospy.Publisher("/ardrone/roll_pid",Vector3,queue_size=1)
		self.pub_alt=rospy.Publisher("/ardrone/alt_pid",Vector3,queue_size=1)
		self.pub_yaw=rospy.Publisher("/ardrone/yaw_pid",Vector3,queue_size=1)
		self.pub_error=rospy.Publisher("/ardrone/error",Twist,queue_size=1)
		self.pub_target=rospy.Publisher("/ardrone/target",Twist,queue_size=1)
		self.pub_state=rospy.Publisher("/ardrone/state",Twist,queue_size=1)
	def TunedParms(self):
		print "pitch Kp",self.pitch_pid.kp,"pitch Kd", self.pitch_pid.kd
		print "roll Kp",self.roll_pid.kp,"roll pitch Kd", self.roll_pid.kd
		print "alt  Kp",self.alt_pid.kp,"alt pitch Kd", self.alt_pid.kd
		print "----------------------------------------"
	def MarkerStateEstimation(self):
		state_x=state_y=state_z=state_yaw=0.0
		marker_flag=False
		alpha=0.8
		ros_image = rospy.wait_for_message("/ardrone/front/image_raw",Image)
		state,orientation=self.marker.RosImageCallBack(ros_image)

		if state!=None:
			orientation=orientation.ravel()

			yaw=orientation[1] #orientation gives -ve values form -3.14 and positive vals from 3.14
			if yaw<0:
				yaw=yaw+3.14
			else:
				yaw=yaw-3.14			
			yaw=(1-alpha)*self.prev_yaw+alpha*yaw #filter
			marker_flag=True	
			state=[state[2],state[0],state[1],yaw]
			self.prev_yaw=yaw
		else:
			marker_flag=False
			state=[0,0,0,0]


		if len(self.state_buffer)<self.buffer_len:
			self.state_buffer.append(state)
		else:
			del(self.state_buffer[0])
			self.state_buffer.append(state)


		temp=[]
		weights=[0.6,0.7,0.8,0.9,1.2]
		for state in self.state_buffer:
			if 	state[0]!=0:
				temp.append(state)

		length=len(temp)
		weights=weights[-length:]
		#for i in range((11-length),11,1):
		#	weights.append(i*0.1) #adding weights 0.1 0.2 0.3 .... 1
			
		if len(temp)>0:
			for j in range(length):#weighted averaging the states
				state_x+=((temp[j][0]*weights[j])/sum(weights))
				state_y+=((temp[j][1]*weights[j])/sum(weights))
				state_z+=((temp[j][2]*weights[j])/sum(weights))
				state_yaw+=((temp[j][3]*weights[j])/sum(weights))
				
		self.avg_state=[state_x,state_y,state_z,state_yaw]

		self.pub_state.publish(Twist(Vector3(state_x,state_y,state_z),Vector3(0,0,state_yaw)))

		return self.avg_state,marker_flag
	def AltitudeData(self):
		odom_data=rospy.wait_for_message('/ardrone/odometry',Odometry)
		z=odom_data.pose.pose.position.z
		return z
	def MarkerTracking(self):		
		state,marker_flag=self.MarkerStateEstimation()
		x,y,z,yaw=state[0],state[1],state[2],state[3]
		target=self.target
		self.pub_target.publish(Twist(Vector3(target[0],target[1],target[2]),Vector3(0,0,0)))
		self.marker_flag=marker_flag
		self.state=[x,y,z,yaw]
		self.error=[x-target[0],
					y-target[1],
					z-target[2],
					yaw-target[3]]

		self.pub_error.publish(Twist(Vector3(x-target[0],y-target[1],z-target[2]),Vector3(0,0,yaw-target[3])))
		dt=rospy.get_rostime().to_sec()-self.now

		#alt_data=self.AltitudeData()
		#alt_input=self.alt_pid.PIDController(z,target[2],dt)
		#if abs(self.alt_pid.error)<0.1:
		#	alt_input=0

		if x!=0 and y!=0:
			pitch_input=self.pitch_pid.PIDController(x,target[0],dt)
			roll_input=self.roll_pid.PIDController(y,target[1],dt)
			alt_input=self.alt_pid.PIDController(z,target[2],dt)
			yaw_input=self.yaw_pid.PIDController(yaw,target[3],dt)
			if abs(self.pitch_pid.error)<0.1:
				pitch_input=0
			if abs(self.roll_pid.error)<0.1:
				roll_input=0
			if abs(self.alt_pid.error)<0.1:
				alt_input=0
			if abs(self.yaw_pid.error)<0.1:
				yaw_input=0
		else:
			alt_input=pitch_input=roll_input=yaw_input=0

		self.pubcmd.publish(Twist(Vector3(-pitch_input*0.35,roll_input*0.35,alt_input),Vector3(0,0,yaw_input*0)))
		#self.pubcmd.publish(Twist(Vector3(-pitch_input*0,roll_input*0,alt_input*0),Vector3(0,0,yaw_input*0.5)))
		self.now=rospy.get_rostime().to_sec()
		self.pub_pitch.publish(Vector3(self.pitch_pid.kp,0,self.pitch_pid.kd))
		self.pub_roll.publish(Vector3(self.roll_pid.kp,0,self.roll_pid.kd))
		self.pub_alt.publish(Vector3(self.alt_pid.kp,0,self.alt_pid.kd))
		self.pub_alt.publish(Vector3(self.yaw_pid.kp,0,self.yaw_pid.kd))
	def DroneTakeoff(self):
		rospy.sleep(1.0)
		self.pubTakeoff.publish(Empty())
		rospy.sleep(5.0)
	def DroneLand(self):
		self.pubLand.publish(Empty())
	def DroneHovering(self):
		self.pubcmd.publish(Twist(Vector3(0,0,0),Vector3(0,0,0.0)))




'''if __name__=='__main__':

	rospy.init_node("AutoPID",anonymous=True)
	pid=AutoPilot()
	rospy.sleep(1.0)
	pid.pubTakeoff.publish(Empty())
	rospy.sleep(5.0)
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		pid.MarkerTracking()
		rate.sleep()
	pid.pubLand.publish(Empty())'''
