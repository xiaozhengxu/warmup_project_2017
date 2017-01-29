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
        rospy.init_node('wall_approach')
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('visualization_marker',Marker, queue_size = 10)
        self.front_distance = None
        self.left_front1 = None
        self.left_front2 = None
        self.left_front3 = None
        self.left_mid = None
        self.left_back1 = None
        self.left_back2 = None
        self.left_back3 = None
        self.target = rospy.get_param('~target_distance')
        self.k = rospy.get_param('~k')
        self.kd = rospy.get_param('~kd')
        self.ka = rospy.get_param('~ka')
        self.found_wall = False
        self.follow_wall = False

    def process_scan(self, msg):
        if msg.ranges[1] != 0.0 or msg.ranges[0]!=0.0 or msg.ranges[360]!=0.0:
            self.front_distance = (msg.ranges[0]+msg.ranges[1]+msg.ranges[360])/3.0
        else:
            self.front_distance = 10.0
        if msg.ranges[65]!=0.0:
            self.left_front1 = msg.ranges[65]
        else:
            self.left_front1 = 10.0
        if msg.ranges[66]!=0.0:
            self.left_front2 = msg.ranges[66]
        else:
            self.left_front2 = 10.0
        # if msg.ranges[60]!=0.0:
        #     self.left_front3 = msg.ranges[75]
        if msg.ranges[90]!=0.0:   
            self.left_mid = msg.ranges[90]
        else:
            self.left_mid = 10.0
        # if msg.ranges[105]!=0.0:   
        #     self.left_back1 = msg.ranges[105]
        if msg.ranges[116]!=0.0:
            self.left_back2 = msg.ranges[116]
        else:
            self.left_back2 = 10.0
        if msg.ranges[115]!=0.0:
            self.left_back3 = msg.ranges[115]
        else:
            self.left_back3 = 10.0

        # if (self.left_front1!=None and self.left_front2!=None and self.left_back2!=None and self.left_back3!=None):
        my_marker = Marker(header = Header(frame_id = "base_link"), scale = Vector3(x = 0.1), points = (
        
        Point(x = math.sin(math.radians(25))*(self.left_front1+self.left_front2)/2,y = math.cos(math.radians(25))*(self.left_front2+self.left_front1)/2),
        # Point(x = math.sin(math.pi/3)*self.left_front2,y = math.cos(math.pi/3)*self.left_front2),
        # # Point(x = math.sin(5*math.pi/12)*self.left_front3,y = math.cos(5*math.pi/12)*self.left_front3),
        # # Point(x = math.sin(7*math.pi/12)*self.left_back1,y = math.cos(7*math.pi/12)*self.left_back1),
        # Point(x = math.sin(2*math.pi/3)*self.left_back2,y = math.cos(2*math.pi/3)*self.left_back2),
        Point(x = 0, y = self.left_mid),
        Point(x = -math.sin(math.radians(25))*(self.left_back2+self.left_back3)/2, y = math.cos(math.radians(25))*(self.left_back2+self.left_back3)/2)), 

        type = 4, color = ColorRGBA(g = 1, a = 1))

        self.vis_pub.publish(my_marker)

    def run(self):
        print 'running'
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.found_wall == False:
                if self.front_distance != None:
                    if (self.front_distance > (self.target+.05) or self.front_distance < (self.target-.05)):
                        error = self.front_distance - self.target
                        self.pub.publish(Twist(linear=Vector3(x=error*self.k)))
                    else:
                        self.pub.publish(Twist(linear=Vector3(x=0.0),angular = Vector3(z = 0)))
                        self.found_wall = True
                        print 'found wall'
            # elif self.follow_wall == False:
            #     if (self.left_mid > (self.target+0.05)) or (self.left_mid < (self.target-0.05)):
            #         error = self.left_mid - self.target
            #         self.pub.publish(Twist(angular=Vector3(z=(-1*error*self.kd))))
            #     else:
            #         self.follow_wall = True
            #         self.pub.publish(Twist(angular=Vector3(z=0)))
            #         print 'in direction to following wall'
            else:
                if self.left_mid != None:
                    errord = self.left_mid - self.target
                    errora = self.left_back3 - self.left_front1 # we're not really using this error now
                    self.pub.publish(Twist(linear = Vector3(x = 0.3), angular = Vector3(z = errord*self.kd+errora*self.ka)))
                    print "following wall"
            r.sleep()
    #k = 0.5, kd = 0.5, ka = 0.1
    # a better constant: kd = 0.2, ka = 0

if __name__ == '__main__':
    node = WallApproach()
    node.run()