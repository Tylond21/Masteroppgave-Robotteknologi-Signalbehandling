#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform
import numpy as np
import matplotlib.pyplot as plt


egm_time_step = 0.004

rospy.init_node("robot_traj_generator")
pub = rospy.Publisher("/robot_set_point", MultiDOFJointTrajectoryPoint, queue_size=1)
rospy.sleep(2)

# start and end of the incline
start_pose = MultiDOFJointTrajectoryPoint()

# get the current robot pose
robot_feedback = MultiDOFJointTrajectoryPoint()
robot_feedback = rospy.wait_for_message("/robot_feedback", MultiDOFJointTrajectoryPoint)
print (robot_feedback)


robot_set_point = robot_feedback


y_start = robot_set_point.transforms[0].translation.y
y_goal = -500.0 # mm
v_max = 2000.0
t_final = abs(y_goal - y_start) / v_max

num_steps = int((t_final)/egm_time_step)
y_arr = np.linspace(y_start, y_goal, num_steps)
t_arr = np.linspace(0, t_final, num_steps)
# plt.plot(t_arr, y_arr)
# plt.show()

for y_set_points in y_arr:
    robot_set_point.transforms[0].translation.y = y_set_points
    pub.publish(robot_set_point)
    rospy.sleep(egm_time_step)
