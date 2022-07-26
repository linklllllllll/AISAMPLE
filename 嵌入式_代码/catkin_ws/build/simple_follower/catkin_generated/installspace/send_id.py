import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import *
import sys, select, termios, tty




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
    room_x = float(room_data[num-1][0])
    room_y = float(room_data[num-1][1])
    room_z = float(room_data[num-1][2])
    room_w = float(room_data[num-1][3])
    print(room_x)
    print(room_y)
    print(room_z)
    print(room_w)
    pose = PoseStamped() #创建目标点对象
    pose.header.frame_id = 'map' #以哪一个TF坐标为原点
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = room_x#目标点位置
    pose.pose.position.y = room_y #目标点位置
    pose.pose.orientation.z = room_z #四元数，到达目标点后小车的方向，z=sin(angle/2)
    pose.pose.orientation.w = room_w #四元数，到达目标点后小车的方向，w=cos(angle/2)
    goal_pub.publish(pose)

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

def breakkey():
    fd = sys.stdin.fileno()
    new_settings = termios.tcgetattr(fd)
    new_settings[3]=new_settings[3] | termios.ECHO
    termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)


if __name__ == "__main__":
    rospy.init_node("send_id")
    settings = termios.tcgetattr(sys.stdin) #获取键值初始化
    rospy.on_shutdown(breakkey)#退出前执行键值初始化
    ID_pub = rospy.Publisher("/ID",Int32,queue_size=1)
    count = 0
    while not rospy.is_shutdown():
        key = getKey() #获取键值
        if(key=='c'): #键值为c是清空目标点
            Id = Int32()
            Id.data = count
            count += 1
            ID_pub.publish(Id)
        elif (key == 'd'):
            count -= 1
            Id = Int32()
            Id.data = count
            ID_pub.publish(Id)
        elif (key == '\x03'): #ctrl+c退出
            break