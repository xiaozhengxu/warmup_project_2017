#!/usr/bin/env python

""" This is a ROS node that follows a person using proportional control """

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Pose,Point, PoseWithCovariance
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

class PersonFollow(object):
    def __init__(self):
        #Set up node, sub and pubs
        rospy.init_node('person_follow')
        rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('visualization_marker',Marker, queue_size = 10)
        #get parameters for proportional control
        self.target = rospy.get_param('~target_distance')
        self.kd = rospy.get_param('~kd')
        self.ka = rospy.get_param('~ka')
        
        #Good paremeters:
        # target distance = 1, kd = 0.5-1, ka = 0.5-1

        self.found_master = False # True if the robot found a person in its view

    def process_scan(self,msg):
        """Processes the laser scan message and finds the center of mass of the person, and publishes that position as a 
        visualization marker
        min_front_index: index in msg.ranges with the minimum distance
        min_front_dist: closest front distance
        person_center_x: x position of center of mass of person
        person_center_y: y position of center of mass of person
        self.person_center_angle: angle of center of mass of person
        self.person_center_dist: distance to center of mass of person
        person_x = []: List of x position of all the points belonging to the person
        person_y = []: List of y position of all the points belonging to the person
        """
        min_front_index = None 
        min_front_dist = 4.5 #Initial miminim front distance
        person_center_x = None 
        person_center_y = None 
        self.person_center_angle = None 
        self.person_center_dist = None
        self.found_master = False
        person_x = [] 
        person_y = []
        
        #Find the closest point in front of the robot:
        for i in range(0,45):
            if msg.ranges[i]>0.3 and msg.ranges[i]<min_front_dist: 
                min_front_dist = msg.ranges[i]
                min_front_index = i
        for j in range(315,361): 
            if msg.ranges[j]>0.3 and msg.ranges[j]<min_front_dist:
                min_front_dist = msg.ranges[j]
                min_front_index = j
                
        #Accumulate all the scan points belonging to the person
        if min_front_dist<3.0:
            person_x.append(min_front_dist*math.cos(math.radians(float(min_front_index))))
            person_y.append(min_front_dist*math.sin(math.radians(float(min_front_index))))
            
            #Look at the 20 points around the closest point, and add them to the list if they are close enough
            i = 0
            while i<10:
                i+=1
                al = min_front_index +i
                ar = min_front_index -i
                #adjust al and ar so they are between 0-360 degrees 
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
                if dl!=0.0 and math.fabs(dl-min_front_dist)<0.1:
                    person_x.append(dl*math.cos(math.radians(al)))
                    person_y.append(dl*math.sin(math.radians(al)))
                if dr!=0.0 and math.fabs(dr-min_front_dist)<0.1:
                    person_x.append(dr*math.cos(math.radians(ar)))
                    person_y.append(dr*math.sin(math.radians(ar)))
            #Find the center of mass by finding the averages of each list x and y
            person_center_x = sum(person_x)/len(person_x)
            person_center_y = sum(person_y)/len(person_y)
            
            #Convert to angle and distance
            self.person_center_angle = math.degrees(math.atan2(person_center_y,person_center_x))
            self.person_center_dist = math.sqrt(person_center_x**2+person_center_y**2)
            
            #Publish visualization marker of the center of mass
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
            '''Publish Twist message to cmd_vel based on distance and angle to person center of mass
            errord: error in distance to person (want to keep this distance self.target)
            errora: error in angle to person (want to keep this angle 0, directly in front of person)
            '''
            if self.found_master:
                if self.person_center_dist!=None: 
                    errord = self.person_center_dist - self.target
                else:
                    errord = 0
                if self.person_center_angle!= None and self.person_center_angle>300: #the angle is closer to 361
                    errora = self.person_center_angle -361  
                elif self.person_center_angle!=None and self.person_center_angle<70: #the angle is closer to 0
                    errora = self.person_center_angle - 0
                else:
                    errora = 0
#                 helps to debug:
#                 print "errora:", errora
#                 print "errord:", errord
                #Publish twist messages:
                if errora!=None and errord!=None:
                    self.pub.publish(Twist(linear=Vector3(x = errord*self.kd), angular = Vector3(z = math.radians(errora)*self.ka)))
                else:
                    self.pub.publish(Twist(linear=Vector3(x= 0), angular = Vector3(z = 0)))
            else: 
                self.pub.publish(Twist(linear=Vector3(x= 0), angular = Vector3(z = 0))) #If no person is found, stop in place
if __name__ == '__main__':
    node = PersonFollow()
    node.run()
