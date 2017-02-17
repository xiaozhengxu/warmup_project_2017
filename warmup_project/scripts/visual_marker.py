#!/usr/bin/env python

from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
import rospy
"""
Create a visualization marker at a specific point that can be viewed in rviz
"""
# initialize vis marker node
rospy.init_node('vis_marker')
# publish Marker message to visualization_marker
pub = rospy.Publisher('visualization_marker',Marker, queue_size = 10)

# Create a marker at point (1,2) in the odometry frame
my_marker = Marker(header = Header(frame_id = "odom"), pose = Pose(position = Point(x = 1, y = 2)))        

r = rospy.Rate(10)

while not rospy.is_shutdown():
    # publsih the marker if not shut down
    pub.publish(my_marker)

