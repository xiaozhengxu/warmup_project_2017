#!/usr/bin/env python
""" This is a ROS node that approaches a wall using proportional control """

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose,Point, PoseWithCovariance
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

class ObstacleAvoid(object):
    def __init__(self):
        rospy.init_node('obstacle_avoid')
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('visualization_marker',Marker, queue_size = 10)

        self.kd = rospy.get_param('~kd')
        self.ka = rospy.get_param('~ka')
        self.init_x = None
        self.init_y = None
        self.init_yaw = None
        self.cur_x = None
        self.cur_y = None
        self.cur_yaw = None
        self.dtheta = None
        self.rt_d = None

        # target distance = 1, kd = 0.5-1, ka = 0.5-1

    def process_odom(self,msg):
        cur_pos = convert_pose_to_xy_and_theta(msg.pose.pose)
        self.cur_x = cur_pos[0]
        self.cur_y = cur_pos[1]
        self.cur_yaw = cur_pos[2]
        if self.init_x == None or self.init_y == None:
            self.init_x = self.cur_x 
            self.init_y =  self.cur_y

    def process_scan(self,msg):
        """Find the closest obstacles in front of the robot:"""
        self.dx = self.init_x - self.cur_x
        self.dy = self.init_y - self.cur_y
        if self.dx <0.1 and self.dy<0.1:
            self.dx = 0
            self.dy = 0
        # self.dx = 0
        # self.dy = 0
        if self.dx > 4:
            self.dx = 4
        if self.dy > 4:
            self.dy = 4
        #dtheta = atan2((self.init_y - self.cur_y)/(self.init_x - self.cur_x))
        # found_obstacle = False
        for i, d in enumerate(msg.ranges):
            if d < 2 and d!=0.0:
                # if found_obstacle == False:
                #     found_obstacle = True
                self.dx += -1*d*math.cos(math.radians(i))
                self.dy += -1*d*math.sin(math.radians(i))
        # for i, d in enumerate(msg.ranges[315:361]):
        #     if d < 2 and d!=0.0:
        #         # if found_obstacle == False:
        #         #     found_obstacle = True
        #         self.dx += -1*d*math.cos(math.radians(i))
        #         self.dy += -1*d*math.sin(math.radians(i))

        # if self.dx == 0 and self.dy == 0:
        #     self.dx = self.init_x - self.cur_x
        #     self.dy = self.init_y - self.cur_y
        #     print "no obstacles"

        # print "found obstacle:", found_obstacle

        self.dtheta = math.atan2(self.dy,self.dx)
        # self.rt_d = math.sqrt(self.dx**2 + self.dy**2)

        # goal_marker = Marker(header = Header(frame_id = "odom"), scale = Vector3(x = 0.1, y = 0.1, z = 0.1), 
        # pose = Pose(position = Point(x = self.init_x+3, y = self.init_y+3)), 
        # type = 2, color = ColorRGBA(g = 1, a = 1))
        # self.vis_pub.publish(goal_marker)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.dtheta!=None:
                self.pub.publish(Twist(linear=Vector3(x = 0.3), angular = Vector3(z = self.dtheta*self.ka)))
                print "self.dx:", self.dx
                print "self.dy:", self.dy
                print "self.dtheta:", self.dtheta

if __name__ == '__main__':
    node = ObstacleAvoid()
    node.run()