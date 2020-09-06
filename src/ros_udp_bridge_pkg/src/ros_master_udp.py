#!/usr/bin/env python3
import rospy
import socket
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from std_srvs.srv import SetBool
import _thread
from RoboticArmClass import RoboticArm

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address_pal = ('192.168.0.205', 9090)
server_address_ang = ('192.168.0.93',9090)
server_address_light_pal = ('localhost',8888)
server_address_light_ang = ('localhost',8888)
#server_address = ('192.168.0.208', 9090)
#sock.bind(server_address)
gripper = '0'
vacuum = '0'
pal_status = -1
ang_status = -1
pal_point = 'p:200:0:200:0'
ang_point = 'g:250:250:100:0'
pal_light = ['l1','1','1','1','1','#']
ang_light = ['l2','1','1','1','1','#']

def read_udp_feedback(thread_name, delay):
    global pal_status,ang_status
    while True:
        data, address = sock.recvfrom(4)
        if not data == None:
            data = data.decode()
            if(data[0] == 'p'):
                pal_status = int(data[2:])
            if(data[0] == 'g'):
                ang_status = int(data[2:])
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
    vacuum = str(int(msg.data))
    str_cmd = pal_point +':'+vacuum+'#'
    sent = sock.sendto(str.encode(str_cmd), server_address_pal)
    rospy.sleep(1)
    return True, 'Success'

def angle_point_remap(msg):
    global ang_status, ang_point
    cmd = list(map(float,msg.point.split()))
    roboticArm = RoboticArm()
    availJointState,goalJointState = roboticArm.InversProblem(cmd[0],cmd[1],cmd[2],-1.57,cmd[3])
    if(not availJointState):
        return False
    str_cmd = 'g:' + msg.point.replace(' ',':')
    ang_point = str_cmd
    str_cmd = str_cmd +':'+gripper+'#'
    sent = sock.sendto(str.encode(str_cmd), server_address_ang)
    rospy.sleep(1)
    result = False
    while True:
        if(ang_status == 0):
            result = True
            ang_status = -1
            break
        else:
            rospy.sleep(0.2)
    return result

def angle_gripper_remap(msg):
    global gripper
    gripper = str(int(not msg.data))
    str_cmd = ang_point +':'+gripper+'#'
    sent = sock.sendto(str.encode(str_cmd), server_address_ang)
    rospy.sleep(1)
    return True, 'Success'

def set_ang_red(msg):
    global ang_light
    ang_light[1] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(ang_light)),server_address_light_ang)
    return True, 'success'

def set_ang_green(msg):
    global ang_light
    ang_light[3] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(ang_light)),server_address_light_ang)
    return True, 'success'

def set_ang_yellow(msg):
    global ang_light
    ang_light[2] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(ang_light)),server_address_light_ang)
    return True, 'success'

def set_ang_blue(msg):
    global ang_light
    ang_light[4] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(ang_light)),server_address_light_ang )
    return True, 'success'

def set_pal_red(msg):
    global pal_light
    pal_light[1] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(pal_light)),server_address_light_pal)
    return True, 'success'

def set_pal_green(msg):
    global pal_light
    pal_light[3] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(pal_light)),server_address_light_pal)
    return True, 'success'

def set_pal_yellow(msg):
    global pal_light
    pal_light[2] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(pal_light)),server_address_light_pal)
    return True, 'success'

def set_pal_blue(msg):
    global pal_light
    pal_light[4] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(pal_light)),server_address_light_pal)
    return True, 'success'

if __name__ == '__main__':
    rospy.init_node('ros_udp')
    _thread.start_new_thread( read_udp_feedback, ("Thread-1", 0.1))
    rospy.Service('/palletizer_robot/cmd_point',point_cmd,palletizer_point_remap)
    rospy.Service('/palletizer_robot/vacuum',SetBool,palletizer_vacuum_remap)
    rospy.Service('/angle_robot/cmd_point',point_cmd,angle_point_remap)
    rospy.Service('/angle_robot/gripper_cmd',SetBool,angle_gripper_remap)
    rospy.Service('/angle_robot/red_light',SetBool,set_ang_red)
    rospy.Service('/angle_robot/green_light',SetBool,set_ang_green)
    rospy.Service('/angle_robot/yellow_light',SetBool,set_ang_yellow)
    rospy.Service('/angle_robot/blue_light',SetBool,set_ang_blue)
    rospy.Service('/palletizer_robot/red_light',SetBool,set_pal_red)
    rospy.Service('/palletizer_robot/green_light',SetBool,set_pal_green)
    rospy.Service('/palletizer_robot/yellow_light',SetBool,set_pal_yellow)
    rospy.Service('/palletizer_robot/blue_light',SetBool,set_pal_blue)
    rospy.spin()