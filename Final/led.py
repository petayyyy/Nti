import rospy
from clover.srv import SetLEDEffect

#rospy.init_node('flight')

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service

def led(color): # 0=yellow, 1=green, 2=blue, 3=red, -1=norhing
    red, green, blue = 0, 0, 0
    if color == 0:
        red, green, blue = 255, 255, 0
    elif color == 1:
        red, green, blue = 0, 255, 0
    elif color == 2:
        red, green, blue = 0, 0, 255
    elif color == 3:
        red, green, blue = 255, 0, 0
    elif color == -1:
        red, green, blue = 0, 0, 0
        
    set_effect(r=red, g=green, b=blue)
