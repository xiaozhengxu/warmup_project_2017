#!/usr/bin/env python
""" This is a ROS node that approaches a wall using proportional control """

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose,Point, PoseWithCovariance
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

class PersonFollow(object):
    def __init__(self):
        rospy.init_node('person_follow')
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('visualization_marker',Marker, queue_size = 10)

        self.target = rospy.get_param('~target_distance')
        self.kd = rospy.get_param('~kd')
        self.ka = rospy.get_param('~ka')

        self.found_master = False

    def process_scan(self,msg):
        min_front_index = None 
        min_front_dist = 4.5
        person_center_x = None
        person_center_y = None
        self.person_center_angle = None
        self.person_center_dist = None
        self.found_master = False
        person_x = []
        person_y = []
        """Find the closest point in front of the robot:"""
        for i in range(0,45):
            if msg.ranges[i]>0.5 and msg.ranges[i]<min_front_dist:
                min_front_dist = msg.ranges[i]
                min_front_index = i
        for j in range(315,361):
            if msg.ranges[j]>0.5 and msg.ranges[j]<min_front_dist:
                min_front_dist = msg.ranges[j]
                min_front_index = j
        """Accumulate all the scan points belonging to the person"""
        if min_front_dist<4.0:
            person_x.append(min_front_dist*math.cos(math.radians(float(min_front_index))))
            person_y.append(min_front_dist*math.sin(math.radians(float(min_front_index))))

            print min_front_index
            print min_front_dist

            i = 0
            while i<21:
                i+=1
                al = min_front_index +i
                ar = min_front_index -i
                if al>360:
                    al-=360
                elif al<0:
                    al+=360
                if ar>360:
                    ar-=360
                elif ar<0:
                    ar+=360
                dl = msg.ranges[al]
                dr = msg.ranges[ar]
                if dl!=0.0 and math.fabs(dl-min_front_dist)<0.3:
                    person_x.append(dl*math.cos(math.radians(al)))
                    person_y.append(dl*math.sin(math.radians(al)))
                if dr!=0.0 and math.fabs(dr-min_front_dist)<0.3:
                    person_x.append(dr*math.cos(math.radians(ar)))
                    person_y.append(dr*math.sin(math.radians(ar)))

            person_center_x = sum(person_x)/len(person_x)
            person_center_y = sum(person_y)/len(person_y)
            
            self.person_center_angle = math.atan2(person_center_y,person_center_x)
            self.person_center_dist = math.sqrt(person_center_x**2+person_center_y**2)

            print "angle:", self.person_center_angle
            print "distance:", self.person_center_dist

            my_marker = Marker(header = Header(frame_id = "base_link"), scale = Vector3(x = 0.1, y = 0.1, z = 0.1), 
            pose = Pose(position = Point(x = person_center_x, y = person_center_y)), 

            type = 2, color = ColorRGBA(g = 1, a = 1))

            self.vis_pub.publish(my_marker)

            self.found_master = True

        else:
            self.found_master = False

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.found_master:
                errord = self.person_center_dist - self.target
                if self.person_center_angle!= None and self.person_center_angle>300:
                    errora = self.person_center_angle -361
                elif self.person_center_angle!=None and self.person_center_angle<70:
                    errora = self.person_center_angle - 0
                else:
                    errora = 0
                self.pub.publish(Twist(linear=Vector3(x = errord*self.kd), angular = Vector3(z = errora*self.ka)))
            else:
                self.pub.publish(Twist(linear=Vector3(x= 0), angular = Vector3(z = 0)))

if __name__ == '__main__':
    node = PersonFollow()
    node.run()