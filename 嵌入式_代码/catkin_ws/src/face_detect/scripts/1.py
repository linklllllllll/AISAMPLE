#!/usr/bin/env python
import socket
from std_msgs.msg import Int32
import rospy

 
rospy.init_node('ID_publisher')
 
# BEGIN PUB
pub = rospy.Publisher('ID', Int32,queue_size=1)

HOST_IP = "192.168.0.101"  # 主机作为AP热点的ip地址
HOST_PORT = 7654  # 端口号
 
print("Starting socket: TCP...")
socket_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建socket
 
print("TCP server listen @ %s:%d!" % (HOST_IP, HOST_PORT))
host_addr = (HOST_IP, HOST_PORT)
socket_tcp.bind(host_addr)  # 绑定主机的ip地址和端口号
socket_tcp.listen(1)  # listen函数的参数是监听客户端的个数，这里只监听一个，即只允许与一个客户端创建连接
 
while True:
    print('waiting for connection...')
    socket_con, (client_ip, client_port) = socket_tcp.accept()  # 接收客户端的请求
    print("Connection accepted from %s." % client_ip)
 
    send_str = "this is string example....wow!!!"
    send_byte=send_str.encode()
    socket_con.send(send_byte)  # 发送数据
 
    while True:
        data = socket_con.recv(1024)  # 接收数据
        data1 = data.decode("utf-8")
        if data==b'1' :
            data2=1
        if data==b'0' :
            data2=0
        if data==b'2' :
            data2=2
        id_msg = Int32()
        id_msg.data = data2
         
        pub.publish(id_msg)
        if data:  # 如果数据不为空，则打印数据，并将数据转发给客户端
            print(data)
            socket_con.send(data)
         
    
            
socket_tcp.close()
