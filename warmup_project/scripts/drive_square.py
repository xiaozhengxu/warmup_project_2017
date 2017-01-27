#!/usr/bin/env python

"""Script to drive the neato in a square """

from geometry_msgs.msg import Twist, Vector3, Point, PoseWithCovariance, Pose 
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import rospy

class Drive_square(object):

	def __init__(self):
		rospy.init_node('drive_square')
		rospy.Subscriber('/odom', Odometry, self.process_odom)
		self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
		self.r = rospy.Rate(10)
		self.initial_pos = None

	def process_odom(self,msg):
		if self.initial_pos == None:
			self.initial_pos = msg.pose.pose.position # point object

# in classroom: turning to the left 
# facing front wall orientation.z: 0.69, w: 0.7188
# facing left wall orientation.z: 0.99, w: 0.032
# facing back wall orientation.z: 0.7315, w:-0.6818
# facing right wall orientation.z: 0.0946 , w: -0.996
	
	def run(self):
		while not rospy.is_shutdown():
			linear_msg = Vector3(x = 0)
			angular_msg = Vector3(z = 0)
			twist_msg = Twist(linear=linear_msg, angular = angular_msg)
			self.pub.publish(twist_msg)
			print twist_msg
			self.r.sleep()

if __name__ == '__main__':
	my_stop = Emergency_Stop()
	my_stop.run()