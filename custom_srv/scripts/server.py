#!/usr/bin/env python

import rospy
import rospkg
from custom_srv.srv import new, newRequest, newResponse
from geometry_msgs.msg import Twist
import sys

def my_callback(request):
    pub = rospy.Publisher('/cmd_vel', Twist)
    print ("Name1: " + request.name1)
    print ("Name2: " + request.name2)
    full_name = request.name1 + " " + request.name2
    my_response = newResponse()
    my_response.fullname = full_name
    
    return my_response

def security_server():
    rospy.init_node('service_server')
    my_service = rospy.Service('/name_confirmation', new, my_callback)
    rospy.spin()

if __name__ == "__main__":
    security_server()