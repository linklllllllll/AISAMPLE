import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from move_base_msgs.msg import *
import sys, select, termios, tty
import subprocess
from std_msgs.msg import Int8
import time
import os



class launch_demo:
    def __init__(self, cmd=None):
        self.cmd = cmd
 
    def launch(self):
        self.child = subprocess.Popen(self.cmd)
        return True
 
    def shutdown(self):
        self.child.terminate()
        self.child.wait()
        return False


def pose_callback(msg):
    # global launch_face,launch_track
    if msg.status.status == 3:
        # if first:
        # os.system("gnome-terminal -- roslaunch face_detect dep02.launch")
        # launch_face.launch()
        time.sleep(3)
        os.system("gnome-terminal -- roslaunch simple_follower face_track.launch")
        # launch_track.launch()

def track_callback(msg):
    # global launch_face,launch_track
    if msg.data == 1:
        # launch_face.shutdown()
        # launch_track.shutdown()
        os.system("gnome-terminal -- ekill face_track.launch")
        os.system("gnome-terminal -- roslaunch gluon_planning follow_face.launch")
        # launch_detect.launch()

def sample_done_callback(msg):
    if msg.data == 1:
        os.system("gnome-terminal -- ekill follow_face.launch")
        # os.system("gnome-terminal -- ekill dep02.launch")
        


#获取键值函数
def getKey():
    fd = sys.stdin.fileno()
    new_settings = termios.tcgetattr(fd)
    new_settings[3]=new_settings[3] | termios.ECHO
    try:
        # termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
        # tty.setraw(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
    return key


def send_goal():
    global count
    rospy.init_node('send_goal') #初始化节点
    goal_pub    = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1) #用于发布目标点
    print("11111111111111111111111111")
    goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, pose_callback) #用于订阅是否到达目标点状态
    track_done_sub = rospy.Subscriber('/track_face_done', Int8, track_callback) #用于订阅是否到达目标点状态
    sample_done_sub = rospy.Subscriber('/sample_done', Int8, sample_done_callback) #用于发布采样完成信号
    

    while not rospy.is_shutdown():
        key = getKey() #获取键值
        if(key=='c'): #键值为c是清空目标点
            pose = PoseStamped() #创建目标点对象
            pose.header.frame_id = 'map' #以哪一个TF坐标为原点
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = count*0.1#目标点位置
            pose.pose.position.y = count*0.1 #目标点位置
            pose.pose.orientation.z = 0 #四元数，到达目标点后小车的方向，z=sin(angle/2)
            pose.pose.orientation.w = 1 #四元数，到达目标点后小车的方向，w=cos(angle/2)
            goal_pub.publish(pose)
            count += 1
        elif (key == '\x03'): #ctrl+c退出
            break

def breakkey():
    fd = sys.stdin.fileno()
    new_settings = termios.tcgetattr(fd)
    new_settings[3]=new_settings[3] | termios.ECHO
    termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin) #获取键值初始化
    rospy.on_shutdown(breakkey)#退出前执行键值初始化
    count = 0
    first = True
    launch_face = launch_demo(["roslaunch", "face_detect", "dep02.launch"])
    launch_track = launch_demo(["rosrun", "simple_follower", "face_track.py"])
    launch_detect = launch_demo(["rosrun","gluon_planning","follow_face.py"])
    send_goal()
