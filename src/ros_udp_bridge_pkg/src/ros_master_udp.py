#!/usr/bin/env python3
import rospy
import socket
import _thread
import os
import json
from math import pi
import time

from inverse_problem_srv.srv import point_cmd
from std_srvs.srv import SetBool

from RoboticArmClassAngle import RoboticArm as RoboticArmAngle
from RoboticArmClassPalletizer import RoboticArm as RoboticArmPalletizer


class AngleRobotControl():
    def __init__(self, cmd_sock, address) -> None:

        rospy.Service('/angle_robot/cmd_point',point_cmd,self.armCmdCb)
        rospy.Service('/angle_robot/gripper_cmd',SetBool,self.gripperCmdCb)
        self.armSolver = RoboticArmAngle()
        self.current_point = [190, 0, 0, 100]
        self.current_gripper = 0
        self.cmd_sock = cmd_sock
        self.address = address
        self.feedbackData(True)
        self.status = 0
        self.sendArmCmd(self.current_point)
        self.last_status_update = time.time()
        rospy.Timer(rospy.Duration(1.0),self.checkArm)

    def armCmdCb(self, msg) -> bool:
        try:
            cmd = list(map(float,msg.point.split(' ')))
            if len(cmd) != 4:
                raise Exception("Invalid count of coordinate")
        except Exception as e:
            rospy.logerr('Invalide command. Error {0}'.format(e))
            return False
        
        rospy.loginfo('New command for angle robot: {0}'.format(msg.point))

        cmd[3] = cmd[3]*180.0/pi

        result, _ = self.armSolver.InversProblem(cmd[0], cmd[1], cmd[2], -1.57, cmd[3])
        if not result:
            rospy.loginfo('No solution can"t be find for point{0}'.format(msg.point))

        cmd[2], cmd[3] = cmd[3], cmd[2]
        
        self.sendArmCmd(cmd)
        self.status = 1
        rospy.sleep(0.3)
        while self.status == 1:
            rospy.sleep(0.5)
            rospy.loginfo("Wait...")

        rospy.sleep(1.0)
        self.current_point = cmd
        return True

    def gripperCmdCb(self, msg)->bool:
        cmd = msg.data
        self.sendGripperCmd(cmd)

        rospy.sleep(0.5)
        while self.status == 1:
            rospy.sleep(0.3)
            rospy.loginfo("Wait...")

        self.current_gripper = int(cmd)
        return True, 'Success'

    def sendArmCmd(self, cmd)->None:
        cmd_srt_list = list(map(str,cmd+[self.current_gripper]))
        cmd_srt = 'g:' + ':'.join(cmd_srt_list) + '#'
        self.cmd_sock.sendto(cmd_srt.encode(), self.address)

    def sendGripperCmd(self, cmd)->None:
        if not cmd:
            gripperCmd='1'
        else:
            gripperCmd='0'
        
        cmd_srt_list = list(map(str,self.current_point+[gripperCmd]))
        cmd_srt = 'g:' + ':'.join(cmd_srt_list) + '#'
        self.cmd_sock.sendto(cmd_srt.encode(), self.address)

    def setArmStatus(self, status)->None:
        self.status = status
        self.last_status_update = time.time()

    def feedbackData(self, flag) -> None:
        if flag:
            cmd = 'r'
        else:
            cmd = 's'
        self.cmd_sock.sendto(cmd.encode(), self.address)

    def checkArm(self, event) -> None:
        if ((time.time() - self.last_status_update)>5.0):
            rospy.logerr('Angle manipulator not respose')
            self.feedbackData(True)


class PalletizerRobotControl():
    def __init__(self, cmd_sock, address) -> None:

        rospy.Service('/palletizer_robot/cmd_point',point_cmd,self.armCmdCb)
        rospy.Service('/palletizer_robot/vacuum',SetBool,self.gripperCmdCb)
        self.armSolver = RoboticArmPalletizer()
        self.current_point = [190, 0, 100] # change
        self.current_gripper = 0
        self.cmd_sock = cmd_sock
        self.address = address
        self.feedbackData(True)
        self.status = 0
        self.sendArmCmd(self.current_point)
        self.last_status_update = time.time()
        rospy.Timer(rospy.Duration(1.0),self.checkArm)

    def armCmdCb(self, msg) -> bool:
        try:
            cmd = list(map(float,msg.point.split(' ')))
            if len(cmd) != 3:
                raise Exception("Invalid count of coordinate")
        except Exception as e:
            rospy.logerr('Invalide command. Error {0}'.format(e))
            return False
        
        rospy.loginfo('New command for palletizer robot: {0}'.format(msg.point))

        result, _ = self.armSolver.InversProblem(cmd[0], cmd[1], cmd[2])
        if not result:
            rospy.loginfo('No solution can"t be find for point{0}'.format(msg.point))
        
        self.sendArmCmd(cmd)
        self.status = 1
        rospy.sleep(0.3)
        while self.status == 1:
            rospy.sleep(0.5)
            rospy.loginfo("Wait...")

        rospy.sleep(1.0)
        self.current_point = cmd
        return True

    def gripperCmdCb(self, msg)->bool:
        cmd = msg.data
        self.sendGripperCmd(cmd)

        rospy.sleep(0.5)
        while self.status == 1:
            rospy.sleep(0.3)
            rospy.loginfo("Wait...")

        rospy.sleep(0.5)
        self.current_gripper = int(cmd)
        return True, 'Success'

    def sendArmCmd(self, cmd)->None:
        cmd_srt_list = list(map(str,cmd+[self.current_gripper]))
        cmd_srt = 'p:' + ':'.join(cmd_srt_list) + '#'
        self.cmd_sock.sendto(cmd_srt.encode(), self.address)

    def sendGripperCmd(self, cmd)->None:
        if cmd:
            gripperCmd='1'
        else:
            gripperCmd='0'
        
        cmd_srt_list = list(map(str,self.current_point+[gripperCmd]))
        cmd_srt = 'p:' + ':'.join(cmd_srt_list) + '#'
        self.cmd_sock.sendto(cmd_srt.encode(), self.address)

    def setArmStatus(self, status)->None:
        self.status = status
        self.last_status_update = time.time()

    def feedbackData(self, flag) -> None:
        if flag:
            cmd = 'r'
        else:
            cmd = 's'
        self.cmd_sock.sendto(cmd.encode(), self.address)

    def checkArm(self, event) -> None:
        if ((time.time() - self.last_status_update)>5.0):
            rospy.logerr('Palletizer manipulator not respose')
            self.feedbackData(True)


class LightControl():
    def __init__(self, robot_name, sock, address) -> None:
        rospy.Service('/'+robot_name+'/red_light',SetBool,self.setRedLightStateCb)
        rospy.Service('/'+robot_name+'/green_light',SetBool,self.setGreenLightStateCb)
        rospy.Service('/'+robot_name+'/yellow_light',SetBool,self.setYellowLightStateCb)
        rospy.Service('/'+robot_name+'/blue_light',SetBool,self.setBlueLightStateCb)

        self.sock = sock
        self.address = address
        self.light_state = ['l','1','0','0','0','#']

    def setRedLightStateCb(self, msg):
        self.light_state[1] = str(int(msg.data))
        self.sendLightState()
        return True, 'success'

    def setGreenLightStateCb(self, msg):
        self.light_state[3] = str(int(msg.data))
        self.sendLightState()
        return True, 'success'

    def setYellowLightStateCb(self, msg):
        self.light_state[4] = str(int(msg.data))
        self.sendLightState()
        return True, 'success'

    def setBlueLightStateCb(self, msg):
        self.light_state[2] = str(int(msg.data))
        self.sendLightState()
        return True, 'success'
    
    def sendLightState(self):
        self.sock.sendto(str.encode(':'.join(self.light_state)),self.address)


def read_udp_feedback(angle_arm_controller, palletizer_arm_controller, address_dict) -> None:

    while True:
        data, address = sock.recvfrom(512)
        if not data == None:
            data = data.decode()
            if(address == address_dict['ang_address']):
                status = int(data.split(':')[2])
                angle_arm_controller.setArmStatus(status)
            if(address == address_dict['pal_address']):
                status = int(data.split(':')[2])
                palletizer_arm_controller.setArmStatus(status)
        rospy.sleep(0.1)

def init_udp_param() -> dict:
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
    return {'pal_address':server_address_pal,\
            'ang_address':server_address_ang,\
            'pal_light_address':server_address_light_pal,\
            'ang_light_address':server_address_light_ang,\
            'server_address':server_address}

if __name__ == '__main__':
    rospy.init_node('ros_udp')

    address_dict = init_udp_param()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(address_dict['server_address'])

    angle_arm_controller = AngleRobotControl(sock, address_dict['ang_address'])
    palletizer_arm_controller = PalletizerRobotControl(sock, address_dict['pal_address'])

    angle_light_control = LightControl('angle_robot', sock, address_dict['ang_light_address'])
    palletizer_light_control = LightControl('palletizer_robot', sock, address_dict['pal_light_address'])

    _thread.start_new_thread(read_udp_feedback, (angle_arm_controller, palletizer_arm_controller, address_dict))

    print('Connected to:', address_dict)

    rospy.spin()
