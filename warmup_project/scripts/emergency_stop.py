#!/usr/bin/env python

"""In class day 3 stopping the robot if an obstacle is detected"""

from geometry_msgs.msg import Twist, Vector3 
from neato_node.msg	import Bump
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import rospy

class Emergency_Stop(object):
"""
This class is defined to detect information from the bump sensor and stop if it is triggered.
"""
	def __init__(self):
		# initialize emergency stop node
		rospy.init_node('e_stop')
		# subscribe to bump sensor and laser scan inputs
		rospy.Subscriber('/bump',Bump, self.process_bump)
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		# publish to Twist messages to command velocity
		self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
		# initialize important values
		self.is_bumped = False 
		self.see_obstacle = False
		self.r = rospy.Rate(10)

	# process bump sensor info to determine if triggered or not
	def process_bump(self,msg):
		if (msg.leftFront==1 or msg.leftFront==1 or msg.rightFront ==1 or msg.rightSide==1):
			self.is_bumped = True
		else:
			self.is_bumped = False

	# process laser scan info to detect obstacles in robot's path
	def process_scan(self,msg):
		for dist in msg.ranges[0:10]: #first ten degrees clockwise from the front of the robot
			if dist == 0.0:
				continue
			if dist < 1.0:
				self.see_obstacle = True
				return
		for dist in msg.ranges[350:361]: #first ten degrees counterclockwise from the front of the robot
			if dist == 0.0:
				continue
			if dist < 1.0:
				self.see_obstacle = True
				return
		self.see_obstacle = False	
	
	#run function that stops the robot if bump sensor is triggered
	def run(self):
		while not rospy.is_shutdown():
			linear_msg = Vector3(x = 0)
			angular_msg = Vector3(z = 0)
			twist_msg = Twist(linear=linear_msg, angular = angular_msg)
			if self.is_bumped:
				twist_msg.linear.x = 0
				self.pub.publish(twist_msg)
			print twist_msg
			self.r.sleep()

if __name__ == '__main__':
	my_stop = Emergency_Stop()
	my_stop.run()
