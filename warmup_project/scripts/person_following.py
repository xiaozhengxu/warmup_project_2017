#!/usr/bin/env python
""" This is a ROS node that approaches a wall using proportional control """

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose,Point, PoseWithCovariance
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

class PersonFollow(object):
	def __init__(self):
		rospy.init_node('person_follow')
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('visualization_marker',Marker, queue_size = 10)

        self.target = rospy.get_param('~target_distance')
        self.kd = rospy.get_param('~kd')
        self.ka = rospy.get_param('~ka')

        self.min_front_index = None 
        self.min_front_dist = 3.0
        self.person_center_x = None
        self.person_center_y = None
        self.person_center_angle = None
        self.person_center_dist = None
        self.person_x = []
        self.person_y = []

        self.found_master = False

    def process_scan(self,msg):
    	"""Find the closest point in front of the robot:"""
    	for i in range(0,61):
    		if msg.ranges[i]!= 0.0 and msg.ranges[i]<self.min_front_dist:
    			self.min_front_dist = msg.ranges[i]
    			self.min_front_index = i
    	for j in range(300,361):
    		if msg.ranges[i]!= 0.0 msg.ranges[j]<self.min_front_dist:
    			self.min_front_dist = msg.ranges[j]
    			self.min_front_index = j

    	"""Accumulate all the scan points belonging to the person"""
    	if self.min_front_dist<2.0:
    		self.found_master = True
    		self.person_x.append(self.min_front_dist*math.sin(math.radians(self.min_front_index)))
    		self.person_y.append(self.min_front_dist*math.cos(math.radians(self.min_front_index)))

    	i = 0
    	while i<21:
    		i++
    		dl = msg.ranges[self.min_front_index+i]
    		dr = msg.ranges[self.min_front_index+360-i]
    		if dl!=0.0 and math.abs(dl-self.min_front_dist)<0.3:
    			self.person_x.append(dl*math.sin(math.radians(self.min_front_index+i)))
    			self.person_y.append(dl*math.cos(math.radians(self.min_front_index+i)))
    		if dr!=0.0 and math.abs(dr-self.min_front_dist)<0.3:
    			self.person_x.append(dl*math.sin(math.radians(self.min_front_index+360-i)))
    			self.person_y.append(dl*math.cos(math.radians(self.min_front_index+360-i)))

    	self.person_center_x = sum(self.person_x)/length(self.person_x)
    	self.person_center_y = sum(self.person_y)/length(self.person_y)
    	
    	self.person_center_angle = math.atan(self.person_center_y/self.person_center_x)
    	self.person_center_dist = math.sqrt(self.person_center_x**2+self.person_center_y**2)

    	my_marker = Marker(header = Header(frame_id = "base_link"), scale = Vector3(x = 0.1), 
    	pose = Pose(position = Point(x = self.person_center_x, y = self.person_center_y)), 

        type = 2, color = ColorRGBA(g = 1, a = 1))

        self.vis_pub.publish(my_marker)

    def run(self):
    	r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.found_master:
                errord = self.person_center_dist - self.target
                if self.person_center_angle>300:
                	errora = self.person_center_angle -361
                elif self.person_center_angle<70:
                	errora = self.person_center_angle - 0
                self.pub.publish(Twist(linear=Vector3(x = errord*self.kd), angular = Vector3(z = errora*self.ka)))
            else:
            	self.pub.publish(Twist(linear=Vector3(x= 0), angular = Vector3(z = 0)))

if __name__ == '__main__':
    node = PersonFollow()
    node.run()