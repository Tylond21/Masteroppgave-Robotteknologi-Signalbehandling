#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform

class VelocityProfile():
    def __init__(self, v_max, a_max, t_final, time_step):
        self.v_max = v_max
        self.a_max = a_max
        self.t_final = t_final
        self. time_step = time_step
        self.t_acc = v_max / a_max
        self.t_dec = t_final - self.t_acc
        
        
    def CreateProfile(self):
        v_acc = np.linspace(0.0, self.v_max, int(self.t_acc / self.time_step))
        v_dwell = np.linspace(self.v_max, self.v_max, int((self.t_dec - self.t_acc) / self.time_step))
        v_dec = np.linspace(self.v_max, 0.0, int((self.t_final - self.t_dec) / self.time_step))
        v_arr = np.concatenate((v_acc, v_dwell, v_dec))
        time_arr = np.linspace(0.0, self.t_final, v_arr.shape[0])
        return v_arr, time_arr



rospy.init_node("VelocityProfile")
pub = rospy.Publisher("/robot_set_point", MultiDOFJointTrajectoryPoint, queue_size=1)
rospy.sleep(2)

# get the current robot pose
robot_feedback = MultiDOFJointTrajectoryPoint()
robot_feedback = rospy.wait_for_message("/robot_feedback", MultiDOFJointTrajectoryPoint)
print (robot_feedback)
robot_set_point = robot_feedback


v_max = -2000.0 #mm/s
a_max = -10000.0 #mm/s2
t_final = 0.4 #s
egm_time_step = 0.004 #s

vy = VelocityProfile(v_max, a_max, t_final, egm_time_step)
vy_arr, t_arr = vy.CreateProfile()
plt.plot(t_arr, vy_arr)
plt.show()
for y_set_points in vy_arr:
    robot_set_point.velocities[0].linear.y = y_set_points
    pub.publish(robot_set_point)
    rospy.sleep(egm_time_step)

    
    
