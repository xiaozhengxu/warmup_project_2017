#!/usr/bin/env python

import tty
import select
import sys
import termios

from geometry_msgs.msg import Twist, Vector3 
import rospy

class Teleop(object):
    '''A node to move the neatos around using keyboard'''
    def __init__(self):
        rospy.init_node('teleop')
        self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
        self.key = None #The key that's pressed
        self.settings = termios.tcgetattr(sys.stdin) 
        self.r = rospy.Rate(10)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def run(self):
        while self.key != '\x03': #exit key (cltc-C)
            self.getKey()
            if self.key == 'u':
                self.pub.publish(Twist(linear = Vector3(x = 1.0), angular = Vector3(z = 0.5))) #turn left and move forward
            elif self.key == 'i':
                self.pub.publish(Twist(linear = Vector3(x = 1.0))) #go straight
            elif self.key == 'o':
                self.pub.publish(Twist(linear = Vector3(x = 1.0),angular = Vector3(z = -0.5))) #turn right and move forward
            elif self.key == 'k' or self.key == '\r': #Enter key or 'k' to stop
                self.pub.publish(Twist(linear = Vector3(x = 0.0),angular = Vector3(z = 0.0))) #stop 
            elif self.key == 'j':
                self.pub.publish(Twist(linear = Vector3(x = 0.0),angular = Vector3(z = 0.5))) # turn left
            elif self.key == 'l':
                self.pub.publish(Twist(linear = Vector3(x = 0.0),angular = Vector3(z = -0.5)))# turn right
            elif self.key == 'm':
                self.pub.publish(Twist(linear = Vector3(x = -1.0), angular = Vector3(z = 0.5))) #turn left while going back
            elif self.key == ',':
                self.pub.publish(Twist(linear = Vector3(x = -1.0))) #go backwards
            elif self.key == '.':
                self.pub.publish(Twist(linear = Vector3(x = -1.0),angular = Vector3(z = -0.5))) #turn right while going back

            self.r.sleep()
            print 'is_running'

if __name__ == '__main__':
    my_teleop = Teleop()
    my_teleop.run()
