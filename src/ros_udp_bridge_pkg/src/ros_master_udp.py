#!/usr/bin/env python3
import rospy
import socket
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from std_srvs.srv import SetBool
import _thread

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address_pal = ('192.168.0.205', 9090)
server_address = ('', 9090)
sock.bind(server_address)
vacuum = '0'
pal_status = -1
pal_point = 'p:200:0:200:0'

def read_udp_feedback(thread_name, delay):
    global pal_status
    while True:
        data, address = sock.recvfrom(4)
        if not data == None:
            data = data.decode()
            pal_status = int(data[2:])
        rospy.sleep(delay)

def palletizer_point_remap(msg):
    global pal_status, pal_point
    str_cmd = 'p:' + msg.point.replace(' ',':')
    pal_point = str_cmd
    str_cmd = str_cmd +':'+vacuum+'#'
    sent = sock.sendto(str.encode(str_cmd), server_address_pal)
    rospy.sleep(1)
    result = False
    while True:
        if(pal_status == 1):
            result = True
            pal_status = -1
            break
        if(pal_status == 0):
            result = False
            pal_status = -1
            break
        else:
            rospy.sleep(0.2)
    return result

def palletizer_vacuum_remap(msg):
    global vacuum
    vacuum = msg.point
    str_cmd = pal_point +':'+vacuum+'#'
    sent = sock.sendto(str.encode(str_cmd), server_address_pal)
    rospy.sleep(1)
    return True

if __name__ == '__main__':
    rospy.init_node('ros_udp')
    _thread.start_new_thread( read_udp_feedback, ("Thread-1", 0.1))
    rospy.Service('/palletizer_robot/cmd_point',point_cmd,palletizer_point_remap)
    rospy.Service('/palletizer_robot/vacuum',point_cmd,palletizer_vacuum_remap)
    rospy.spin()