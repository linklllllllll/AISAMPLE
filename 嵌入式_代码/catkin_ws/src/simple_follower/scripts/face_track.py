#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Int8,Int32
import os


arrive_flag=0
lim_linear_x = 0.3
lim_linear_y = 0.1

track_distance = [0.32,0.35,0.35]
track_error = [0.01,0.01,0.01]
track_bias = [0.085,0.085,0.1]

ratio = 0.8
id = 0
start_track = False



class Follower:
    def __init__(self):
        # self.track_start = rospy.Subscriber("/track_start", String, self.track_callback)
        self.face_sub = rospy.Subscriber("/face", PoseStamped, self.face_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.arrival_sub = rospy.Subscriber("/arrived_flag",Int8,self.arrival_callback,queue_size=1)
        self.arrive_pub = rospy.Publisher("/track_face_done",Int8,queue_size=1)
        self.ID_sub = rospy.Subscriber("/ID",Int32,self.ID_callback,queue_size=1)


    def limit(self, x, xmin, xmax):
        if x < xmin:
            x = xmin
        elif x > xmax:
            x = xmax
        return x

    def ID_callback(self, ID):
        global id
        id = ID.data
        rospy.loginfo("now id =" +str(id))
        
    # 机器人已经导航至指定房间 开始跟随人脸
    def arrival_callback(self,msg):
        global start_track, arrive_flag
        if msg.data == 1:
            start_track = True
            arrive_flag = 0
            os.system("aplay -D plughw:CARD=Device,DEV=0 ~/catkin_ws/src/xf_mic_asr_offline/feedback_voice/寻找目标.wav")
            rospy.loginfo("start_track")

    # def track_callback(self, msg):
    #     global arrive_flag
    #     if(msg.data == "start_track_face"):
    #         arrive_flag = 0
    #         rospy.loginfo("starting tracking face.\n")

    def face_callback(self, msg):
        global arrive_flag, lim_linear_x, lim_linear_y, track_distance, track_error, ratio,start_track,id,track_bias
        if start_track:
            distance = msg.pose.position.z
            bias = msg.pose.position.x + track_bias[id]
            if arrive_flag == 0:
                if distance > track_distance[id] or abs(bias) > track_error[id]:
                    if distance > track_distance[id]:
                        self.twist.linear.x = (distance-track_distance[id])*ratio
                        self.twist.linear.x = self.limit(self.twist.linear.x,-lim_linear_x,lim_linear_x)
                    else:
                        self.twist.linear.x = 0
                    if abs(bias) >track_error[id]:
                        if bias<0:
                            self.twist.linear.y = -(bias+track_error[id])*ratio
                        else:
                            self.twist.linear.y = -(bias-track_error[id])*ratio
                        self.twist.linear.y = self.limit(self.twist.linear.y,-lim_linear_y,lim_linear_y)
                    else:
                        self.twist.linear.y = 0
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    arrive_flag = 1
                    rospy.loginfo("Arrived!")
                    arrive_data = Int8()
                    arrive_data.data = 1
                    self.arrive_pub.publish(arrive_data) #跟随人脸结束
                    start_track = False
                    self.twist.linear.x = 0
                    self.twist.linear.y = 0
                    self.cmd_vel_pub.publish(self.twist)
                    # os.system("rosrun gluon_planning follow_face.py")
                rospy.loginfo("distance:%f vel_x:%f",distance,self.twist.linear.x)
                rospy.loginfo("bias:%f vel_y:%f",bias,self.twist.linear.y)
            else:
                self.twist.linear.x = 0
                self.twist.linear.y = 0
                self.cmd_vel_pub.publish(self.twist)

    # def face_callback_x(self, msg):
    #     global arrive_flag
    #     distance = msg.pose.position.z
    #     bias = msg.pose.position.x
    #     if arrive_flag == 0:
    #         if distance > 0.2:
    #             self.twist.linear.x = (distance-0.2)*0.8
    #             self.twist.linear.x = self.limit(self.twist.linear.x,-lim_linear_x,lim_linear_x)
    #             self.twist.linear.y = -bias*0.8
    #             self.twist.linear.y = self.limit(self.twist.linear.y,-lim_linear_y,lim_linear_y)
    #         else:
    #             self.twist.linear.x = 0
    #             self.twist.linear.y = 0
    #             arrive_flag = 1
    #             rospy.loginfo("Arrived!")
    #         self.cmd_vel_pub.publish(self.twist)
    #         rospy.loginfo("distance:%f vel_x:%f",distance,self.twist.linear.x)
    #         rospy.loginfo("bias:%f vel_y:%f",bias,self.twist.linear.y)
    #     else:
    #         self.twist.linear.x = 0
    #         self.twist.linear.y = 0
    #         self.cmd_vel_pub.publish(self.twist)

rospy.init_node("face_track")
follower = Follower()
rospy.spin()

