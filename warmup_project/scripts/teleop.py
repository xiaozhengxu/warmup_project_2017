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
        print "TEST"
        while self.key != '\x03':
            self.getKey()
            print ord(self.key)
            if self.key == chr(27):
                print "escape was pressed"
            self.r.sleep()
            print 'is_running'

if __name__ == '__main__':
    my_teleop = Teleop()
    my_teleop.run()
