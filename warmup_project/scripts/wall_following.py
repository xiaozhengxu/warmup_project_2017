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
                        self.pub.publish(Twist(linear=Vector3(x=0.0)))
                        self.found_wall = True
                        print 'found wall'
            elif self.follow_wall == False:
                if (self.left_mid > (self.target+0.05)) or (self.left_mid < (self.target-0.05)):
                    error = self.left_mid - self.target
                    self.pub.publish(Twist(angular=Vector3(z=(-1*error*self.k))))
                else:
                    self.follow_wall = True
                    print 'following wall'
            # else:

            #     self.pub.publish(Twist(linear = Vector3(x = 1.0), angular = Vector3(z = error*self.k)))


   #          else self.x !=None and self.z!=None and self.done == False:
            #   linear_msg = Vector3(x = self.x)
            #   angular_msg = Vector3(z = self.z)
            #   twist_msg = Twist(linear=linear_msg, angular = angular_msg)
            #   self.pub.publish(twist_msg)
            #   print twist_msg
            # self.r.sleep()
            r.sleep()

if __name__ == '__main__':
    node = WallApproach()
    node.run()