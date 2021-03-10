#!/usr/bin/env python3
import rospy
import socket
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from std_srvs.srv import SetBool
import _thread
import os
import json
from RoboticArmClass import RoboticArm

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address_pal = server_address_ang = server_address_light_pal = server_address_light_ang = server_address = 0
gripper = '0'
vacuum = '0'
ang_status = -1
grip_status = -1
pal_status = -1
vacuum_status = -1
ang_point = '0:190:170:1.57'
pal_point = '0:-250:80:0'
pal_light = ['l','1','0','0','0','#']
ang_light = ['l','1','0','0','0','#']

def read_udp_feedback(thread_name, delay):
    global ang_status,server_address,grip_status,pal_status,vacuum_status
    while True:
        data, address = sock.recvfrom(4)
        if not data == None:
            data = data.decode()
            if(data[0] == 'a'):
                ang_status = int(data[2:])
            if(data[0] == 'g'):
                grip_status = int(data[2:])
            if(data[0] == 'p'):
                pal_status = int(data[2:])
        rospy.sleep(delay)

def angle_point_remap(msg):
    global ang_status, ang_point
    print("new msg for angle: %s"%msg.point)
    cmd = list(map(float,msg.point.split()))
    if len(cmd) < 4:
        raise Exception('Invalid command format to control the angle manipulator')

    roboticArm = RoboticArm()
    availJointState,goalJointState = roboticArm.InversProblem(cmd[0],cmd[1],cmd[2],-1.57,cmd[3])
    if(not availJointState):
        print('unreacheble')
        return False

    str_cmd = msg.point.replace(' ',':')
    if(str_cmd == ang_point):
        return True
    ang_point = str_cmd
    str_cmd = 'a:'+str_cmd
    sent = sock.sendto(str.encode(str_cmd), server_address_ang)
    ang_status = -1
    rospy.sleep(3.0)
    result = False
    start_time = rospy.get_time()
    while True:
        if(ang_status == 0):
            result = True
            ang_status = -1
            break
        if(rospy.get_time()-start_time>10):
            sent = sock.sendto(str.encode(str_cmd), server_address_ang)
            start_time = rospy.get_time()
        else:
            rospy.sleep(0.01)
    return result

def angle_gripper_remap(msg):
    global gripper,grip_status
    gripper_new = str(int(not msg.data))
    print('new gripper cmd: %s'%gripper_new)
    if(gripper_new == gripper):
        return True, 'Success'
    else:
        gripper = gripper_new
    str_cmd = 'g:'+gripper
    sent = sock.sendto(str.encode(str_cmd), server_address_ang)
    grip_status = -1
    rospy.sleep(2.0)
    result = False
    while True:
        if(grip_status == 0):
            result = True
            grip_status = -1
            break
        else:
            rospy.sleep(0.01)
    return True, 'Success'

def palletizer_point_remap(msg):
    global pal_status, pal_point
    print("new palletizer point: %s"%msg.point)
    cmd = list(map(float,msg.point.split()))
    if len(cmd) < 3:
        raise Exception('Invalid command format to control the palletizer')
    str_cmd = msg.point.replace(' ',':')+':0'
    if(pal_point==str_cmd):
        return True
    else:
        pal_point = str_cmd
    str_cmd = "p:" + str_cmd
    sent = sock.sendto(str.encode(str_cmd), server_address_pal)
    rospy.sleep(3.0)
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
    vacuum_new = str(int(msg.data))
    print("new vacuum: %s"%vacuum_new)
    if(vacuum_new == vacuum):
        return True, 'Success'
    else:
        vacuum = vacuum_new
    str_cmd = 'v:'+vacuum
    sent = sock.sendto(str.encode(str_cmd), server_address_pal)
    rospy.sleep(2.0)
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
    ang_light[4] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(ang_light)),server_address_light_ang)
    return True, 'success'

def set_ang_blue(msg):
    global ang_light
    ang_light[2] = str(int(msg.data))
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
    pal_light[4] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(pal_light)),server_address_light_pal)
    return True, 'success'

def set_pal_blue(msg):
    global pal_light
    pal_light[2] = str(int(msg.data))
    sock.sendto(str.encode(':'.join(pal_light)),server_address_light_pal)
    return True, 'success'

def init_udp_param():
    global server_address_pal,server_address_ang,server_address_light_pal,server_address_light_ang,server_address
    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(THIS_FOLDER, 'config_1.json')
    if config_file==None:
        return False
    with open(config_file) as json_file:
        data = json.load(json_file)
        server_address = ('',data['input_server_port'])
        server_address_pal = (data['pal_address'],data['pal_port'])
        server_address_ang = (data['ang_address'],data['ang_port'])
        server_address_light_ang = (data['ang_light_address'],data['ang_light_port'])
        server_address_light_pal = (data['pal_light_address'],data['pal_light_port'])
    return True

if __name__ == '__main__':
    rospy.init_node('ros_udp')
    init_udp_param()
    sock.bind(server_address)
    _thread.start_new_thread( read_udp_feedback, ("read_udp_thread", 0.5))
    print(server_address_pal,server_address_ang,server_address_light_pal,server_address_light_ang,server_address)
    sock.sendto(str.encode(':'.join(pal_light)),server_address_light_pal)
    sock.sendto(str.encode(':'.join(ang_light)),server_address_light_ang )
    sock.sendto(str.encode('a:'+ang_point), server_address_ang)
    sock.sendto(str.encode('p:'+pal_point), server_address_pal)
    sock.sendto(b"v:0", server_address_pal)
    sock.sendto(b"g:0", server_address_ang)
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
