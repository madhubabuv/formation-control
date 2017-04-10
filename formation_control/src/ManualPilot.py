#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Vector3

class ManualPilot:
	def __init__(self):
		#rospy.init_node("ManualPilot",anonymous=True)
		self.pubCommand=rospy.Publisher("/cmd_vel",Twist,queue_size=1)	
	def KeyboradControl(self,control):
		pitch=roll=alt=yaw=0
		if control=='a':
			print 'Going left'
			roll=-0.5
		elif control=='d':
			print 'Going right'
			roll=0.5
		elif control=='w':
			print 'Going up'
			alt=0.5
		elif control=='s':
			print 'Going down'
			alt=-0.5
		elif control=='f':
			print 'Going forward'
			pitch=0.5
		elif control=='b':
			print 'Going backward'
			pitch=-0.5
		elif control=='l':
			print "Turning Left"
			yaw=0.5
		elif control=='r':
			print "Turning Right"
			yaw=-0.5
		else:
			print "Enter correct character"
		self.pubCommand.publish(Twist(Vector3(pitch,roll,alt),Vector3(0,0,yaw)))
		rospy.sleep(0.05)
