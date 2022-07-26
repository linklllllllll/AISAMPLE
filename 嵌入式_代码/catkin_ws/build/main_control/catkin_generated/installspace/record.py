#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Int8
from move_base_msgs.msg import *
import os

class Recorder:
    def __init__(self):
        arrival_sub = rospy.Subscriber('/arrived_flag', Int8, self.arrival_callback) #用于订阅是否到达目标点状态
        track_done_sub = rospy.Subscriber('/track_face_done', Int8, self.track_callback) #用于订阅是否到达目标点状态
        sample_done_sub = rospy.Subscriber('/sample_done', Int8, self.sample_done_callback) #用于发布采样完成信号



    def arrival_callback(self, msg):
        if(msg.data == 1):
            rospy.loginfo("The robot has arrived at the goal.\n")

    def track_callback(self, msg):
        if(msg.data == 1):
            rospy.loginfo("The robot has arrived at the face.\n")

    def sample_done_callback(self, msg):
        if(msg.data == 1):
            rospy.loginfo("Sample Success.\n")


if __name__ == '__main__':
    rospy.init_node("record")
    follower = Recorder()
    rospy.spin()

