#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_matrix
import numpy as np
from math import pi


class Transformation():
    def __init__(self, camera_topic, robot_fb_topic, robot_sp_topic):
        
        # init subscribers
        rospy.Subscriber(camera_topic, Float64MultiArray, self.camera_callback)
        rospy.Subscriber(robot_fb_topic, MultiDOFJointTrajectoryPoint, self.robot_callback)
        
        # init set point publisher
        self.pub = rospy.Publisher(robot_sp_topic, MultiDOFJointTrajectoryPoint, queue_size=1)
         
        # transformation data
        self.follow_offset_z = 450.0
        self.rot_tool_base = [0.0, -0.26958, 0.96298, 0.0]
        self.quat_tool_base = quaternion_matrix([self.rot_tool_base[1], self.rot_tool_base[2], self.rot_tool_base[3], self.rot_tool_base[0]])
        self.trans_tool_base = [0.0, 0.0, 0.0]
        # self.trans_cam_tool = [38.3, 0.0, -45.0]
        self.rot_cam_tool = euler_matrix(0, 0, -99.67*(pi/180.0))
        
        # safety limits for x y z:
        self.x_lim = [-700.0, 200.0]
        self.y_lim = [80.0, 700.0]
        self.z_lim = [100.0, 800.0]

        self.calibration_tool_base = [46.73, 35.54, 0.0]
        
     
    def camera_callback(self, msg):
        
        # ball pose wrt the camera
        ball_cam = np.array([0.0, 0.0, 0.0, 1.0])
        ball_cam[0] = msg.data[0]
        ball_cam[1] = msg.data[1]
        ball_cam[2] = msg.data[2]

        ball_cam_vel = np.array([0.0, 0.0, 0.0, 1.0])
        ball_cam_vel[0] = msg.data[3]
        ball_cam_vel[1] = msg.data[4]
        ball_cam_vel[2] = 0.0
        
        # transform to the tool
        # ball position wrt base link frame
        ball_tool = np.array([0.0, 0.0, 0.0, 1.0])
        ball_tool = np.matmul(self.rot_cam_tool, ball_cam)

        ball_tool_vel = np.array([0.0, 0.0, 0.0, 1.0])
        ball_tool_vel = np.matmul(self.rot_cam_tool, ball_cam_vel)
       
        
        #transform to the base link
        ball_base = np.array([0.0, 0.0, 0.0, 1.0])
        ball_base = np.matmul(self.quat_tool_base, ball_tool)
        ball_base[0] = ball_base[0] + self.trans_tool_base[0]
        ball_base[1] = ball_base[1] + self.trans_tool_base[1]
        ball_base[2] = ball_base[2] + self.trans_tool_base[2]
        
        ball_base_vel = np.array([0.0, 0.0, 0.0, 1.0])
        ball_base_vel = np.matmul(self.quat_tool_base, ball_tool_vel)

        # publish robot set point msg
        robot_set_point_msg = MultiDOFJointTrajectoryPoint()
        robot_set_point_msg.transforms.append(Transform())
        robot_set_point_msg.velocities.append(Twist())
        
        robot_set_point_msg.transforms[0].translation.x = ball_base[0]
        robot_set_point_msg.transforms[0].translation.y = ball_base[1]
        robot_set_point_msg.transforms[0].translation.z = 700.0
        robot_set_point_msg.transforms[0].rotation.x = self.rot_tool_base[1]
        robot_set_point_msg.transforms[0].rotation.y = self.rot_tool_base[2]
        robot_set_point_msg.transforms[0].rotation.z = self.rot_tool_base[3]
        robot_set_point_msg.transforms[0].rotation.w = self.rot_tool_base[0]
        robot_set_point_msg.velocities[0].linear.x = ball_base_vel[0]
        robot_set_point_msg.velocities[0].linear.y = ball_base_vel[1]
        robot_set_point_msg.velocities[0].linear.z = 0.0
        robot_set_point_msg.velocities[0].angular.x = 0.0
        robot_set_point_msg.velocities[0].angular.y = 0.0
        robot_set_point_msg.velocities[0].angular.z = 0.0
        
        if self.consider_set_point(robot_set_point_msg):
            print (robot_set_point_msg)
            self.pub.publish(robot_set_point_msg)
        else:
            print (robot_set_point_msg)
            print ("Invalid set point")

            
    def robot_callback(self, msg):
        # print ("feedback received")
        self.trans_tool_base[0] = msg.transforms[0].translation.x
        self.trans_tool_base[1] = msg.transforms[0].translation.y
        self.trans_tool_base[2] = msg.transforms[0].translation.z
            
            
    def consider_set_point(self, set_point):
        x = set_point.transforms[0].translation.x
        y = set_point.transforms[0].translation.y
        z = set_point.transforms[0].translation.z
        if x>=self.x_lim[0] and x<=self.x_lim[1]:
            if y>=self.y_lim[0] and y<=self.y_lim[1]: 
                if z>=self.z_lim[0] and z<=self.z_lim[1]:
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False
            
rospy.init_node("transformation")
camera_topic = "/vision"
robot_fb_topic = "/robot_feedback"
robot_sp_topic = "/robot_set_point"
Transformation(camera_topic, robot_fb_topic, robot_sp_topic)
while not rospy.is_shutdown():
    pass

