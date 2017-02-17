#!/usr/bin/env python

""" This is one of my recommended methods for implementing
    finite state control in ROS Python
    Note: this is kind of a hodgepodge of actual Python and
          pseudo code, so there may be small typos """
import rospy
import math
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, Pose,Point, PoseWithCovariance
from std_msgs.msg import Header, ColorRGBA, String
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


class FiniteStateController(object):
    """ The intention of this example is to show how you would
        use object-oriented principles to create a finite-state
        controller that wall follows until an person is detected
        in front of the robot, and then attempts to follow that person. """

    # these are constants that let us give a name to each of our states
    # the names don't necessarily have to match up with the names of
    # the methods that are used to implement each behavior.
    PERSON_FOLLOW_STATE = "person_follow"
    OBSTAClE_AVOID_STATE = "obstacle_avoid"
    STOPPED_STATE = "stopped"

    def __init__(self):
        rospy.init_node('state_controller')
        self.state = FiniteStateController.STOPPED_STATE
        
        # subscribe to relevant sensor topics
        rospy.Subscriber('/bump', Bump, self.process_bump)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        
        #publish to topics
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('visualization_marker',Marker, queue_size = 10)
        self.state_pub = rospy.Publisher('state', String, queue_size=10)
        #attributes for person follow
        self.target = rospy.get_param('~target_distance')
        self.kd = rospy.get_param('~kd')
        self.ka = rospy.get_param('~ka')
        self.person_center_angle = None
        self.person_center_dist = None
            # target distance = 1, kd = 0.5-1, ka = 0.5-1

        #attributes for object avoiding
        self.ka_obj_avoid = rospy.get_param('~ka_obj_avoid')
        self.init_x = None
        self.init_y = None
        self.cur_x = None
        self.cur_y = None
        self.dtheta = None
        self.rt_d = None

        #attributes for state change
        self.found_master = False
        self.bumped = False

    def process_bump(self, msg):
        """ Process bump sensor info to determine if triggered or not """
        self.state_pub.publish(String(self.state))
        if (msg.leftFront or
            msg.rightFront or
            msg.rightSide or
            msg.leftSide):
            self.bumped = True

    def process_odom(self,msg):
        """ Determine where the robot is from its starting position """
        cur_pos = convert_pose_to_xy_and_theta(msg.pose.pose)
        self.cur_x = cur_pos[0]
        self.cur_y = cur_pos[1]
        if self.init_x == None or self.init_y == None:
            self.init_x = self.cur_x 
            self.init_y =  self.cur_y

    def process_scan(self, msg):
        '''For person following:'''
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
            if msg.ranges[i]>0.3 and msg.ranges[i]<min_front_dist:
                min_front_dist = msg.ranges[i]
                min_front_index = i
        for j in range(315,361):
            if msg.ranges[j]>0.3 and msg.ranges[j]<min_front_dist:
                min_front_dist = msg.ranges[j]
                min_front_index = j
        """Accumulate all the scan points belonging to the person"""
        if min_front_dist<3.0:
            person_x.append(min_front_dist*math.cos(math.radians(float(min_front_index))))
            person_y.append(min_front_dist*math.sin(math.radians(float(min_front_index))))
            # print "min front index:", min_front_index
            # print "min front distance:", min_front_dist
            i = 0
            while i<10:
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
                if dl!=0.0 and math.fabs(dl-min_front_dist)<0.1:
                    person_x.append(dl*math.cos(math.radians(al)))
                    person_y.append(dl*math.sin(math.radians(al)))
                if dr!=0.0 and math.fabs(dr-min_front_dist)<0.1:
                    person_x.append(dr*math.cos(math.radians(ar)))
                    person_y.append(dr*math.sin(math.radians(ar)))

            person_center_x = sum(person_x)/len(person_x)
            person_center_y = sum(person_y)/len(person_y)
            
            self.person_center_angle = math.degrees(math.atan2(person_center_y,person_center_x))
            self.person_center_dist = math.sqrt(person_center_x**2+person_center_y**2)

            # print "angle:", self.person_center_angle
            # print "distance:", self.person_center_dist

            my_marker = Marker(header = Header(frame_id = "base_link"), scale = Vector3(x = 0.1, y = 0.1, z = 0.1), 
            pose = Pose(position = Point(x = person_center_x, y = person_center_y)), 

            type = 2, color = ColorRGBA(g = 1, a = 1))

            self.vis_pub.publish(my_marker)

            self.found_master = True

        else:
            self.found_master = False

        """Obstacle avoiding:"""
        self.dx = 0
        self.dy = 0
        for i, d in enumerate(msg.ranges):
            if d < 2 and d!=0.0:
                self.dx += -1*d*math.cos(math.radians(i))
                self.dy += -1*d*math.sin(math.radians(i))

        self.dtheta = math.atan2(self.dy,self.dx)

    def obstacle_avoid(self):
        """ Publishes a Twist message to determine the robot's speed and direction unless state must change"""
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.bumped:
                return FiniteStateController.STOPPED_STATE
            elif self.found_master:
                return FiniteStateController.PERSON_FOLLOW_STATE
            else:
                if self.dtheta!=None:
                    self.pub.publish(Twist(linear=Vector3(x = 0.3), angular = Vector3(z = self.dtheta*self.ka)))
                # print "self.dx:", self.dx
                # print "self.dy:", self.dy
                # print "self.dtheta:", self.dtheta

    def person_follow(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            """
            Publish Twist message to cmd_vel based on distance and angle to person center of mass unless state must change
            errord: error in distance to person (want to keep this distance self.target)
            errora: error in angle to person (want to keep this angle 0, directly in front of person)
            """
            if self.bumped:
                return FiniteStateController.STOPPED_STATE
            elif self.found_master:
                if self.person_center_dist!=None:
                    errord = self.person_center_dist - self.target
                else:
                    errord = 0
                if self.person_center_angle!= None and self.person_center_angle>300:
                    errora = self.person_center_angle -361
                elif self.person_center_angle!=None and self.person_center_angle<70:
                    # print "center angle:", self.person_center_angle
                    errora = self.person_center_angle 
                else:
                    errora = 0
                # print "errora:", errora
                # print "errord:", errord
                if errora!=None and errord!=None:
                    self.pub.publish(Twist(linear=Vector3(x = errord*self.kd), angular = Vector3(z = math.radians(errora)*self.ka)))
                else:
                    self.pub.publish(Twist(linear=Vector3(x= 0), angular = Vector3(z = 0)))
            else:
                return FiniteStateController.OBSTAClE_AVOID_STATE

                 
    def stop(self):
        """ Stops the robot if bump sensor is triggered unless state must change """
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x= 0), angular = Vector3(z = 0)))
            if not self.bumped:
                if self.found_master:
                    return FiniteStateController.PERSON_FOLLOW_STATE
                else:
                    return FiniteStateController.OBSTAClE_AVOID_STATE

    def run(self):
        """ Run function that determines which function will run based on the state of the robot """
        while not rospy.is_shutdown():
            # print "self.found master:",self.found_master
            # print "self.bumped:", self.bumped
            """Based on state, run certain behavior"""
            if self.state == FiniteStateController.OBSTAClE_AVOID_STATE:
                self.state = self.obstacle_avoid()
            elif self.state == FiniteStateController.PERSON_FOLLOW_STATE:
                self.state = self.person_follow()
            elif self.state == FiniteStateController.STOPPED_STATE:
                self.state = self.stop()
            else:
                print "invalid state!!!" # note this shouldn't happen

if __name__ == '__main__':
    node = FiniteStateController()
    node.run()
