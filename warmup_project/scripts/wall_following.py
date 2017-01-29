#!/usr/bin/env python
""" This is a ROS node that approaches a wall using proportional control """

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class WallApproach(object):
    def __init__(self):
        rospy.init_node('wall_approach')
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.front_distance = None
        self.left_front = None
        self.left_mid = None
        self.left_back = None
        self.target = rospy.get_param('~target_distance')
        self.k = rospy.get_param('~k')
        self.kd = rospy.get_param('~kd')
        self.ka = rospy.get_param('~ka')
        self.found_wall = False
        self.follow_wall = False

    def process_scan(self, msg):
        if msg.ranges[1] != 0.0 or msg.ranges[0]!=0.0 or msg.ranges[360]!=0.0:
            self.front_distance = (msg.ranges[0]+msg.ranges[1]+msg.ranges[360])/3.0
        if msg.ranges[45]!=0.0:
            self.left_front = msg.ranges[45]
        else:
            self.left_front = 10.0
        if msg.ranges[90]!=0.0:   
            self.left_mid = msg.ranges[90]
        else:
            self.left_mid = 10.0
        if msg.ranges[135]!=0.0:
            self.left_back = msg.ranges[135]
        else:
            self.left_back = 10.0

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.found_wall == False:
                if self.front_distance != None:
                    if (self.front_distance > (self.target+.05) or self.front_distance < (self.target-.05)):
                        error = self.front_distance - self.target
                        self.pub.publish(Twist(linear=Vector3(x=error*self.k)))
                    else:
                        self.pub.publish(Twist(linear=Vector3(x=0.0),angular = Vector3(z = -0.3)))
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
                errord = self.left_mid - self.target
                # errord = 0
                errora = self.left_back - self.left_front
                self.pub.publish(Twist(linear = Vector3(x = 0.5), angular = Vector3(z = errord*self.kd+errora*self.ka)))
                print "following wall"
            r.sleep()
    #k = 0.5, kd = 0.5, ka = 0.1

if __name__ == '__main__':
    node = WallApproach()
    node.run()