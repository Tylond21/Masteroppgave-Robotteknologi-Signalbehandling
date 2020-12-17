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
        # rospy.Subscriber(robot_fb_topic, MultiDOFJointTrajectoryPoint, self.robot_callback)
        
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
        self.jog_scale = 5
        self.new_jog_received = False
        self.jog_tool_vel = np.array([0.0, 0.0, 0.0, 1.0])

        
     
    def camera_callback(self, msg):

        self.jog_tool_vel[0] += msg.data[0] * self.jog_scale
        self.jog_tool_vel[1] += msg.data[1] * self.jog_scale
        self.jog_tool_vel[2] += msg.data[2] * self.jog_scale
        self.new_jog_received = True
           
          
    def jog_robot(self):
        if (self.new_jog_received):
            # self.pub.publish(self.robot_set_point_msg)
            self.new_jog_received = False
        else:
            
            # ramp down
            rospy.sleep(1)
            if(self.jog_tool_vel[0] > 0):
                self.jog_tool_vel[0] += -self.jog_scale

            if(self.jog_tool_vel[0] < 0):
                self.jog_tool_vel[0] += self.jog_scale

            if(self.jog_tool_vel[0] == 0):
                self.jog_tool_vel[0] = 0

            if(self.jog_tool_vel[1] > 0):
                self.jog_tool_vel[1] += -self.jog_scale

            if(self.jog_tool_vel[1] < 0):
                self.jog_tool_vel[1] += self.jog_scale

            if(self.jog_tool_vel[1] == 0):
                self.jog_tool_vel[1] = 0

            if(self.jog_tool_vel[2] > 0):
                self.jog_tool_vel[2] += -self.jog_scale

            if(self.jog_tool_vel[2] < 0):
                self.jog_tool_vel[2] += self.jog_scale

            if(self.jog_tool_vel[2] == 0):
                self.jog_tool_vel[2] = 0

            #transform to the base link
            jog_base_vel = np.array([0.0, 0.0, 0.0, 1.0])
            jog_base_vel = np.matmul(self.quat_tool_base, self.jog_tool_vel)

            #  robot set point msg
            robot_set_point_msg = MultiDOFJointTrajectoryPoint()
            robot_set_point_msg.transforms.append(Transform())
            robot_set_point_msg.velocities.append(Twist())
            robot_set_point_msg.velocities[0].linear.x = jog_base_vel[0]
            robot_set_point_msg.velocities[0].linear.y = jog_base_vel[1]
            robot_set_point_msg.velocities[0].linear.z = jog_base_vel[2]
            robot_set_point_msg.velocities[0].angular.x = 0.0
            robot_set_point_msg.velocities[0].angular.y = 0.0
            robot_set_point_msg.velocities[0].angular.z = 0.0 
            self.pub.publish(robot_set_point_msg)


rospy.init_node("transformation")
camera_topic = "/robot_set_point_jog"
robot_fb_topic = "/robot_feedback"
robot_sp_topic = "/robot_set_point"
trans = Transformation(camera_topic, robot_fb_topic, robot_sp_topic)

while not rospy.is_shutdown():
    trans.jog_robot()
    pass

