import lcm
import time
import numpy as np
import sys
from forwardKinect import *
from inverseKinect import getJointAnglesFromIK
sys.path.append("lcmtypes")

from lcmtypes import dynamixel_command_t
from lcmtypes import dynamixel_command_list_t
from lcmtypes import dynamixel_status_t
from lcmtypes import dynamixel_status_list_t
from lcmtypes import dynamixel_config_t
from lcmtypes import dynamixel_config_list_t

"""Useful Conversions"""
PI = np.pi
D2R = PI/180.0
R2D = 180.0/3.141592
ANGLE_TOL = 2*PI/180.0 


""" Rexarm Class """
class Rexarm():
    def __init__(self):

        """ TODO: modify this class to add functionality you need """

        """ Commanded Values """
        self.num_joints = 6                     # number of motors, increase when adding gripper
        # self.joint_angles = [0.0] * self.num_joints # radians
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, -75*D2R, 0] # radians


        # you must change this to an array to control each joint speed separately 
        self.speed = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]                         # 0 to 1
        # self.speed = [1.0, 0.5, 0.3, 0.5, 0.5, 0.5]                         # 0 to 1
        self.max_torque = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]                    # 0 to 1
        self.speed_multiplier = 0.3
        self.torque_multiplier = 1.0

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # radians
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius               

        """ Waypoint Plan - TO BE USED LATER """
        self.plan = []
        self.plan_status = 0
        self.wpt_number = 0
        self.wpt_total = 0

        """ Setup LCM and subscribe """
        self.lc = lcm.LCM()
        lcmMotorSub = self.lc.subscribe("DXL_STATUS",
                                        self.feedback_handler)

    def cmd_publish(self):
        """ 
        Publish the commands to the arm using LCM.
        This publishes the joint angles currently set in the Rexarm object
        You need to call this function to command the arm.
        """    
        msg = dynamixel_command_list_t()
        msg.len = self.num_joints
        self.clamp()
        
        for i in range(msg.len):
            cmd = dynamixel_command_t()
            cmd.utime = int(time.time() * 1e6)
            cmd.position_radians = self.joint_angles[i]
            cmd.speed = self.speed_multiplier * self.speed[i]
            cmd.max_torque = self.torque_multiplier * self.max_torque[i]
            msg.commands.append(cmd)
        self.lc.publish("DXL_COMMAND",msg.encode())

    def cfg_publish_default(self):
        """ 
        Publish default configuration to arm using LCM. 
        """    
        msg = dynamixel_config_list_t()
        msg.len = self.num_joints
        for i in range(msg.len):
            cfg = dynamixel_config_t()
            cfg.utime = int(time.time() * 1e6)
            # cfg.kp = 32
            # cfg.kd = 0
            # cfg.ki = 0

            cfg.kp = 32
            cfg.kd = 20
            cfg.ki = 20

            cfg.compl_margin = 0
            cfg.compl_s
            lope = 16
            cfg.LED = 1
            msg.configs.append(cfg)
        self.lc.publish("DXL_CONFIG",msg.encode())

    def cfg_publish(self):
        """ 
        TODO: implement this function (optional)

        Publish configuration to arm using LCM. 
        You need to activelly call this function to command the arm.
        """    
        pass
    
    def get_feedback(self):
        """
        LCM Handler function
        Called continuously with timer by control_station.py
        times out after 10ms
        """
        self.lc.handle_timeout(10)

    def feedback_handler(self, channel, data):
        """
        Feedback Handler for LCM
        this is run when a feedback message is recieved
        """
        msg = dynamixel_status_list_t.decode(data)
        for i in range(msg.len):
            self.joint_angles_fb[i] = msg.statuses[i].position_radians 
            self.speed_fb[i] = msg.statuses[i].speed 
            self.load_fb[i] = msg.statuses[i].load 
            self.temp_fb[i] = msg.statuses[i].temperature

    def clamp(self):
        """
        TODO: implement this function

        Clamp Function
        Limit the commanded joint angles to ones physically possible 
        so the arm is not damaged.
        """
        # Clamp for should
        pass
        # if self.joint_angles[1]*R2D > 50:
        #      self.joint_angles[1] = 50*D2R
        # if self.joint_angles[1]*R2D < -50:
        #      self.joint_angles[1] = -50*D2R

        # # Clamp for Elbow
        # if self.joint_angles[2]*R2D > 100:
        #      self.joint_angles[2] = 100*D2R
        # if self.joint_angles[2]*R2D < -100:
        #      self.joint_angles[2] = -100*D2R

        # # Clamp for Wrist
        # if self.joint_angles[3]*R2D > 110:
        #      self.joint_angles[3] = 110*D2R
        # if self.joint_angles[3]*R2D < -110:
        #      self.joint_angles[3] = -110*D2R

    def rexarm_FK(self,dh_table, link):
        """
        TODO: implement this function

        Calculates forward kinematics for rexarm
        takes a DH table filled with DH parameters of the arm
        and the link to return the position for
        returns a 4-tuple (x, y, z, phi) representing the pose of the 
        desired link
        """
        #   ----- +z
        #   |
        #   |
        #  +x

        # 1 - sholder; 2 - elbow; 3 - wrist -radius
        theta1,theta2,theta3, a1, a2, a3, baseTheta = dh_table

        A1 = get_A(np.pi/2-theta1, a1)
        A2 = get_A(-theta2, a2)
        A3 = get_A(-theta3, a3)  

        A = np.dot(np.dot(A1, A2), A3)
        phi = np.pi / 2 - (theta1 + theta2 + theta3)

        # tranform [0,0,0,1] to end effector
        origin = np.array([[0],[0],[0],[1]])
        end_loc = np.dot(A, origin)
        # print('$$$')
 
        # print('pin loc: \n', end_loc)
        # print(end_loc.shape)
        x, y, z = end_loc[0,0], end_loc[1,0], end_loc[2,0]

        # z is determined by the rotation of base motor
        z = x * np.sin(baseTheta)
        x_tick = x * np.cos(baseTheta)
        y_tick = y + 116 # 116 is the dist from z1 to board

        link = (x_tick, y_tick, z, phi)
        return link

    	
    def rexarm_IK(self, pose, cfg):
        """
        TODO: implement this function

        Calculates inverse kinematics for the rexarm
        pose is a tuple (x, y, z, phi) which describes the desired
        end effector position and orientation.  
        cfg describe elbow down (0) or elbow up (1) configuration
        returns a 4-tuple of joint angles or NONE if configuration is impossible
        """
        return getJointAnglesFromIK(pose, cfg)

        
    def rexarm_collision_check(q):
        """
        TODO: implement this function

        Perform a collision check with the ground and the base of the Rexarm
        takes a 4-tuple of joint angles q
        returns true if no collision occurs
        """
        pass 
