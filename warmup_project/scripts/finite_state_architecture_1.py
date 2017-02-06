#!/usr/bin/env python

""" This is one of my recommended methods for implementing
	finite state control in ROS Python
	Note: this is kind of a hodgepodge of actual Python and
		  pseudo code, so there may be small typos """
import rospy
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan

class FiniteStateController(object):
	""" The intention of this example is to show how you would
		use object-oriented principles to create a finite-state
		controller that wall follows until an person is detected
		in front of the robot, and then attempts to follow that person. """

	# these are constants that let us give a name to each of our states
	# the names don't necessarily have to match up with the names of
	# the methods that are used to implement each behavior.
	WALL_FOLLOW_STATE = "wall_follow"
	PERSON_FOLLOW_STATE = "person_follow"

	def __init__(self):
		rospy.init_node('state_controller')
		self.state = FiniteStateController.WALL_FOLLOW_STATE
		# subscribe to relevant sensor topics
		rospy.Subscriber('/bump', Bump, self.process_bump)
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		# setup publishers (e.g. for motor commands)
		# not shown

		# attributes needed for run loop
		self.bumped = False
		self.forward_obstacle_detected = False

	def process_bump(self, msg):
		if (msg.leftFront or
			msg.rightFront or
			msg.rightSide or
			msg.leftSide):
			self.bumped = True

	def process_scan(self, msg):
		if msg.ranges[45] and msg.ranges[135]:
			# enables simple proportional control of angle
			self.distance_diff = msg.ranges[45] - msg.ranges[135]
		# detect whether there is an object in front of us
		self.forward_obstacle_detected = msg.ranges[0] != 0.0

	def wall_follow(self):
		while not rospy.is_shutdown():
			# handle wall following in here
			# turn proportionally to self.distance_diff (not shown)
			if self.bumped:
				self.bumped = False
				return FiniteStateController.PERSON_FOLLOW_STATE

	def person_follow(self):
		while not rospy.is_shutdown():
			# do person following thing here (not shown)
			if not self.forward_obstacle_detected:
				return FiniteStateController.WALL_FOLLOW_STATE

	def run(self):
		while not rospy.is_shutdown():
			if self.state == FiniteStateController.WALL_FOLLOW_STATE:
				self.state = self.wall_follow()
			elif self.state == FiniteStateController.PERSON_FOLLOW_STATE:
				self.state = self.person_follow()
			else:
				print "invalid state!!!" # note this shouldn't happen

if __name__ == '__main__':
	node = FiniteStateController()
	node.run()