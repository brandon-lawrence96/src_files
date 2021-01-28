#!/usr/bin/env python

import rospy
import rospkg
from custom_srv.srv import new, newRequest
from geometry_msgs.msg import Twist
import sys

def concate_name_client(x,y):
    rospy.wait_for_service('/name_confirmation')
    rospy.init_node('service_client')
    robot_service = rospy.ServiceProxy('/name_confirmation', new)
    robot_service_object = newRequest()
    robot_service_object.name1 = x
    robot_service_object.name2 = y
    result = robot_service(robot_service_object)

    return result

def usage():
    return "%s [x,y]"%sys.argv[0]

def move_robot_square():
    pub = rospy.Publisher('/cmd_vel', Twist)
    rospy.sleep(1.0)
    r = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        move = Twist()
        move.linear.x = 0.3
        for i in range(10):
            pub.publish(move)
            r.sleep()

        move = Twist()
        move.angular.z = 1.57/1.88
        for i in range(10):
            pub.publish(move)
            r.sleep()


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = sys.argv[1]
        y =  sys.argv[2]
    else:
        print (usage())
        sys.exit(1)

    result = concate_name_client(x, y)
    if (result.fullname) == "Brandon Lawrence":
        move_robot_square()
    else:
        print("Wrong Person!")


        









