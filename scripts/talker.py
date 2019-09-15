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

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import std_msgs.msg

joints = ['pivot1', 'pivot2', 'pivot3']

pub = None
rate = None

def send(lst):
    head = std_msgs.msg.Header()
    head.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    msg = JointState()
    msg.header = head
    msg.name = joints
    msg.position = lst 
    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()

def talker():
    global pub
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('my_teleop', anonymous=True)
    step = 0.04
    pos = 0.0  
    pos2 = 0.0 
    pos3 = 0.0
    global rate
    rate =rospy.Rate(5)
    for x in range(10):
        lst = [0, 0, pos]
        pos -= step
        send(lst)# 10hz
    for x in range(10):
        lst = [pos2, pos3, pos]
        pos2 += 2*step
        pos3 -= step
        send(lst)# 10hz
        

    while not rospy.is_shutdown():
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
