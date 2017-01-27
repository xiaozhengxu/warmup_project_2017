#!/usr/bin/env python

import tty
import select
import sys
import termios

from geometry_msgs.msg import Twist, Vector3 
import rospy

class Teleop(object):
    def __init__(self):
        print "in here"
        rospy.init_node('teleop')
        self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
        self.key = None
        self.settings = termios.tcgetattr(sys.stdin)
        self.r = rospy.Rate(10)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        #return self.key

    def run(self):
        while self.key != '\x03':
            self.getKey()

            if self.key == 'u':
                self.pub.publish(Twist(linear = Vector3(x = 1.0), angular = Vector3(z = 0.5))) #turn left
            elif self.key == 'i':
                self.pub.publish(Twist(linear = Vector3(x = 1.0)))
            elif self.key == 'o':
                self.pub.publish(Twist(linear = Vector3(x = 1.0),angular = Vector3(z = -0.5)))
            elif self.key == 'k' or self.key == '\r':
                self.pub.publish(Twist(linear = Vector3(x = 0.0),angular = Vector3(z = 0.0)))
            elif self.key == 'j':
                self.pub.publish(Twist(linear = Vector3(x = 0.0),angular = Vector3(z = 0.5)))
            elif self.key == 'l':
                self.pub.publish(Twist(linear = Vector3(x = 0.0),angular = Vector3(z = -0.5)))
            elif self.key == 'm':
                self.pub.publish(Twist(linear = Vector3(x = -1.0), angular = Vector3(z = 0.5))) #turn left
            elif self.key == ',':
                self.pub.publish(Twist(linear = Vector3(x = -1.0)))
            elif self.key == '.':
                self.pub.publish(Twist(linear = Vector3(x = -1.0),angular = Vector3(z = -0.5)))

            self.r.sleep()
            print 'is_running'

if __name__ == '__main__':
    my_teleop = Teleop()
    my_teleop.run()
