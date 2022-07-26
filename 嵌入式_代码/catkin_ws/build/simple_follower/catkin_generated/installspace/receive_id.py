import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import *
import os



def readTxt():
    data = []
    with open("/home/ubuntu/catkin_ws/room.txt","r") as f:
        for line in f.readlines():
            line = line.strip("\n")
            line = line.split()
            data.append(line)
    # print(data)
    return data

def id_callback(msg):
    # print(1)
    num = msg.data
    # print(type(num))
    rospy.loginfo("id: " + str(num))
    room_data = readTxt()
    room_x = float(room_data[num][0])
    room_y = float(room_data[num][1])
    room_o_x = float(room_data[num][2])
    room_o_y = float(room_data[num][3])
    room_z = float(room_data[num][4])
    room_w = float(room_data[num][5])
    print(room_x)
    print(room_y)
    print(room_z)
    print(room_w)
    pose = PoseStamped() #创建目标点对象
    pose.header.frame_id = 'map' #以哪一个TF坐标为原点
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = room_x#目标点位置
    pose.pose.position.y = room_y #目标点位置
    pose.pose.orientation.x = room_o_x #四元数，到达目标点后小车的方向，z=sin(angle/2)
    pose.pose.orientation.y = room_o_y #四元数，到达目标点后小车的方向，z=sin(angle/2)
    pose.pose.orientation.z = room_z #四元数，到达目标点后小车的方向，z=sin(angle/2)
    pose.pose.orientation.w = room_w #四元数，到达目标点后小车的方向，w=cos(angle/2)
    goal_pub.publish(pose)
    if num == 0:
        os.system("aplay -D plughw:CARD=Device,DEV=0 ~/catkin_ws/src/xf_mic_asr_offline/feedback_voice/1号导航.wav")
    elif num == 1:
        os.system("aplay -D plughw:CARD=Device,DEV=0 ~/catkin_ws/src/xf_mic_asr_offline/feedback_voice/2号导航.wav")




if __name__ == "__main__":
    rospy.init_node("receive_id")
    room_num = rospy.Subscriber("/ID",Int32,id_callback)
    goal_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
    rospy.spin()
