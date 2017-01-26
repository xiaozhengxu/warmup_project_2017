import tty
import select
import sys
import termios

from geometry_msgs.msg import Twist, Vector3 
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import rospy


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)
key = None

while key != '\x03':
    key = getKey()
    print key

