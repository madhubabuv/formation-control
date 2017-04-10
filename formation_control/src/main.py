#!/usr/bin/env python
import rospy
import os
from FollowMarkerPilot import AutoPilot
from ManualPilot import ManualPilot
from KeyBoardReader import KBHit

'''-linear.x: move backward
+linear.x: move forward
-linear.y: move right
+linear.y: move left
-linear.z: move down
+linear.z: move up

-angular.z: turn right
+angular.z: turn left'''


if __name__=='__main__':

	#intilization of objects
	rospy.init_node('test1',anonymous=True)
	target=[1.8,0.0,0,0.0]
	alt_target=1.5
	marker_id=97
	marker_size=0.125
	takeoff_flag=True
	kb = KBHit()
	auto_pilot=AutoPilot(target,alt_target,marker_id,marker_size)	
	manual_pilot=ManualPilot()

	rate=rospy.Rate(10)	
	while not rospy.is_shutdown():
		if kb.kbhit():
			main_input=kb.getch()
			if main_input=="m":
				print '''
			Altitude Up :w
			Altitude Down :s
			Left :a
			Right :d
			Front :f
			Back  :b
			quit Manual Mode : 'q' '''
				key=''
				while key!='q':
					if kb.kbhit():
						key=kb.getch()
						manual_pilot.KeyboradControl(key)
					else:
						auto_pilot.DroneHovering()
				print "\tExiting Manual Mode\n\tAutonomous Mode : n \n \tManual Mode m\n"
			elif main_input=='t':
				if takeoff_flag:
					print "\nDrone Take off"
					auto_pilot.DroneTakeoff()
					takeoff_flag=False
				else:
					print "Drone has been already taken off"
			elif main_input=='e':
				print "\n....Drone Landing"
				auto_pilot.DroneLand()
				print "Exiting Program"
				exit()
			else:
				print "Enter Correct Charcter"


			#print pilot.pitch,pilot.roll,pilot.alt
		else:
			auto_pilot.MarkerTracking()	
			if not takeoff_flag:	
				os.system('clear')
				print "\n\n\t\t\tAuto Mode Active \n \t\t\tPress 'e' Emergency Landing"
				print "Marker Status :",auto_pilot.marker_flag
				print "\t\t X   :",round(auto_pilot.state[0],3),"\t\t Target X   :",target[0],"\t\t error X   :",round(auto_pilot.error[0],3)
				print "\t\t Y   :",round(auto_pilot.state[1],3),"\t\t Target Y   :",target[1],"\t\t error Y   :",round(auto_pilot.error[1],3)
				print "\t\t Z   :",round(auto_pilot.state[2],3),"\t\t Target Z   :",target[2],"\t\t error Z   :",round(auto_pilot.error[2],3)
				print "\t\t Yaw :",round(auto_pilot.state[3],3),"\t\t Target Yaw :",target[3],"\t\t error Yaw :",round(auto_pilot.error[3],3)
			else:
				os.system('clear')
				print "Please take off the Drone\n\t\t....Press 't' to takeoff"
		rate.sleep()



	
