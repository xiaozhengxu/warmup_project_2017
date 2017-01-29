#!/usr/bin/env python

"""Script to drive the neato in a square """

from geometry_msgs.msg import Twist, Vector3, Point, PoseWithCovariance, Pose 
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import rospy
import math

class Drive_square(object):

	def __init__(self):
		rospy.init_node('drive_square')
		rospy.Subscriber('/odom', Odometry, self.process_odom)
		self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
		self.r = rospy.Rate(10)
		self.x = None
		self.z = None
		self.state = 'turn'
		self.last_pos = None
		self.initial_pos = None
		self.pos_2 = None
		self.pos_3 = None
		self.pos_4 = None
		self.done = False

	def process_odom(self,msg):
		orien_z = msg.pose.pose.orientation.z
		orien_w = msg.pose.pose.orientation.w
		if self.last_pos!=None:
			dist_to_last_pos = math.sqrt((msg.pose.pose.position.x-self.last_pos.x)**2+(msg.pose.pose.position.y-self.last_pos.y)**2)		
		if self.initial_pos == None:
			self.initial_pos = msg.pose.pose.position # point object
			self.last_pos = self.initial_pos
		elif self.pos_2 == None:
			if not (orien_z<0.73 and orien_z>0.65 and orien_w<0.75 and orien_w>0.67):
				self.z = 0.3
				self.x = 0
			elif dist_to_last_pos<1: #if the orientation is right, and not yet travelled 1m, move forward
				self.z = 0
				self.x = 1
			else: # if it has travelled 1m, if has reached the second position
				self.pos_2 = msg.pose.pose.position
				self.last_pos = self.pos_2
				self.z = 0
				self.x = 0
				print "pos_2 reached"
		elif self.pos_3 == None:
			if not (orien_z<1.03 and orien_z>0.95 and orien_w<0.07 and orien_w>-0.9):
				self.z = 0.3
				self.x = 0
			elif dist_to_last_pos<1: #if the orientation is right, and not yet travelled 1m, move forward
				self.z = 0
				self.x = 1
			else: # if it has travelled 1m, if has reached the third position
				self.pos_3 = msg.pose.pose.position
				self.last_pos = self.pos_3
				self.z = 0
				self.x = 0
				print "pos_3 reached"
		elif self.pos_4 == None:
			if not (orien_z<0.77 and orien_z>0.69 and orien_w<-0.64 and orien_w>-0.72):
				self.z = 0.3
				self.x = 0
			elif dist_to_last_pos<1: #if the orientation is right, and not yet travelled 1m, move forward
				self.z = 0
				self.x = 1
			else: # if it has travelled 1m, if has reached the fourth position
				self.pos_4 = msg.pose.pose.position
				self.last_pos = self.pos_4
				self.z = 0
				self.x = 0
				print "pos_4 reached"
		else: # It has reached the 4th position, go back to the initial position 
			if not (orien_z<0.14 and orien_z>0.06 and orien_w<-0.94 and orien_w>-1.03):
				self.z = 0.3
				self.x = 0
			elif dist_to_last_pos<1: #if the orientation is right, and not yet travelled 1m, move forward
				self.z = 0
				self.x = 1
			else: # if it has travelled 1m, if has reached the initial position
				self.z = 0
				self.x = 0
				print "initial posistion reached"
				self.last_pos = self.initial_pos
				linear_msg = Vector3(x = self.x)
				angular_msg = Vector3(z = self.z)
				twist_msg = Twist(linear=linear_msg, angular = angular_msg)
				self.pub.publish(twist_msg)
				self.done = True

# in classroom: turning to the left 
# facing front wall orientation.z: 0.69, w: 0.7188
# facing left wall orientation.z: 0.99, w: 0.032
# facing back wall orientation.z: 0.7315, w:-0.6818
# facing right wall orientation.z: 0.0946 , w: -0.996
	
	def run(self):
		while not rospy.is_shutdown():
			if self.x !=None and self.z!=None and self.done == False:
				linear_msg = Vector3(x = self.x)
				angular_msg = Vector3(z = self.z)
				twist_msg = Twist(linear=linear_msg, angular = angular_msg)
				self.pub.publish(twist_msg)
				# print twist_msg
			self.r.sleep()

if __name__ == '__main__':
	ds = Drive_square()
	ds.run()
