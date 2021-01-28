#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(data):
    #convert range to numpy array
    laser_range = np.array(data.ranges)
    #remove +/- inf entries from array
    laser_range = laser_range[~np.isinf(laser_range)]
    laser_max = max(laser_range)
    if laser_max < 1.2:
        turn()
        move_forward()
        #laser_max = 1.0
    
    move_forward()

def turn():
    move = Twist()
    move.angular.z = 1.57/2
    rate = rospy.Rate(10) # 10hz
    for i in range(1):
        pub.publish(move)
        rate.sleep()

    print("robot turning")

def move_forward():
    move = Twist()
    move.linear.x = 0.6
    pub.publish(move)

    print("robot moving forward")

def main():
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()        

rospy.init_node('avoider', anonymous=True)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
