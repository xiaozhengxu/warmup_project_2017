#!/usr/bin/env python
""" This is a ROS node that approaches a wall using proportional control """

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose,Point, PoseWithCovariance
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

class WallApproach(object):
    def __init__(self):
        #set up subs and pubs
        rospy.init_node('wall_approach')
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('visualization_marker',Marker, queue_size = 10)
        #initializes variables
        self.front_distance = None
        self.left_front1 = None
        self.left_front2 = None
        self.left_mid = None
        self.left_back1 = None
        self.left_back2 = None
        #get parameters for proportional control
        self.target = rospy.get_param('~target_distance')
        self.k = rospy.get_param('~k')
        self.kd = rospy.get_param('~kd')
        self.ka = rospy.get_param('~ka')
        
        self.found_wall = False 
        self.follow_wall = False 

    def process_scan(self, msg):
        """This function processes the laser scan message and in particular updates 3 key values on the left side of the robot 
        and one value directly in front of the robot. Because the laser scan values are finicky, 
        for each key angle there are 2 neiboring laser measurements being used. """
        #Update the distance on the front to find wall
        if msg.ranges[0] != 0.0:
            self.front_distance = msg.ranges[0]
        elif msg.ranges[1]!=0.0:
            self.front_distance = msg.ranges[1]
        elif msg.ranges[360]!=0.0:
            self.front_distance = msg.ranges[360]
        else:
            self.front_distance = 10.0
        #Update distances on the left
        if msg.ranges[65]!=0.0:
            self.left_front1 = msg.ranges[65]
        else:
            self.left_front1 = 10.0
        if msg.ranges[66]!=0.0:
            self.left_front2 = msg.ranges[66]
        else:
            self.left_front2 = 10.0
        if msg.ranges[90]!=0.0:   
            self.left_mid = msg.ranges[90]
        else:
            self.left_mid = 10.0
        if msg.ranges[116]!=0.0:
            self.left_back2 = msg.ranges[116]
        else:
            self.left_back2 = 10.0
        if msg.ranges[115]!=0.0:
            self.left_back1 = msg.ranges[115]
        else:
            self.left_back1 = 10.0

        #publish a line marker that should be the wall based on the three measurements on the left
        my_marker = Marker(header = Header(frame_id = "base_link"), scale = Vector3(x = 0.1), points = (
        Point(x = math.sin(math.radians(25))*(self.left_front1+self.left_front2)/2,y = math.cos(math.radians(25))*(self.left_front2+self.left_front1)/2),
        Point(x = 0, y = self.left_mid),
        Point(x = -math.sin(math.radians(25))*(self.left_back2+self.left_back1)/2, y = math.cos(math.radians(25))*(self.left_back2+self.left_back1)/2)), 
        type = 4, color = ColorRGBA(g = 1, a = 1))
        self.vis_pub.publish(my_marker)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.found_wall == False:
                #Find wall, assuming that the robot head straight into the wall. 
                if self.front_distance != None:
                    if (self.front_distance > (self.target+.05) or self.front_distance < (self.target-.05)):
                        error = self.front_distance - self.target
                        self.pub.publish(Twist(linear=Vector3(x=error*self.k)))
                    else:
                        self.pub.publish(Twist(linear=Vector3(x=0.0),angular = Vector3(z = 0)))
                        self.found_wall = True
                        print 'found wall'
            else: 
                #Following wall
                errord = self.left_mid - self.target
                errora = self.left_back3 - self.left_front1 # we're not really using this error now by setting ka = 0, because that seems to work better
                self.pub.publish(Twist(linear = Vector3(x = 0.3), angular = Vector3(z = errord*self.kd+errora*self.ka)))
                print "following wall"
            r.sleep()
            #k = 0.5, kd = 0.5, ka = 0.1
            # a better constant: kd = 0.2, ka = 0

if __name__ == '__main__':
    node = WallApproach()
    node.run()
