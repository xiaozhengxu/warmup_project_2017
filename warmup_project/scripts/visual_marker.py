#!/usr/bin/env python

from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
import rospy

rospy.init_node('vis_marker')
pub = rospy.Publisher('visualization_marker',Marker, queue_size = 10)

my_marker = Marker(header = Header(frame_id = "odom"), pose = Pose(position = Point(x = 1, y = 2)))        

r = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(my_marker)

