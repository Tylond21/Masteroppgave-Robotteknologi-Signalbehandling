#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, Transform
import numpy as np
import matplotlib.pyplot as plt


egm_time_step = 0.04

def interpolate(start, end, step_size=egm_time_step, t_final=4):
    num_steps = int((t_final)/step_size)
    inter_arr = np.linspace(start, end, num_steps)
    t_arr = np.linspace(0, t_final, num_steps)
    return t_arr, inter_arr
    

rospy.init_node("robot_traj_generator")
pub = rospy.Publisher("/robot_set_point", MultiDOFJointTrajectoryPoint, queue_size=1)
rospy.sleep(2)

start_incline = MultiDOFJointTrajectoryPoint()
start_incline.transforms.append(Transform())
start_incline.velocities.append(Twist())
x, y, z = -540.67, 168.28, 700.0
q1, q2, q3, q4 = 0.0, -0.26958, 0.96298, 0.0

start_incline.transforms[0].translation.x = x
start_incline.transforms[0].translation.y = y
start_incline.transforms[0].translation.z = z
start_incline.transforms[0].rotation.x = q2
start_incline.transforms[0].rotation.y = q3
start_incline.transforms[0].rotation.x = q4
start_incline.transforms[0].rotation.w = q1

end_incline = MultiDOFJointTrajectoryPoint()
end_incline.transforms.append(Transform())
end_incline.velocities.append(Twist())
x, y, z = 156.46, 644.71, 650.0
q1, q2, q3, q4 = 0.0, -0.26958, 0.96298, 0.0

end_incline.transforms[0].translation.x = x
end_incline.transforms[0].translation.y = y
end_incline.transforms[0].translation.z = z
end_incline.transforms[0].rotation.x = q2
end_incline.transforms[0].rotation.y = q3
end_incline.transforms[0].rotation.x = q4
end_incline.transforms[0].rotation.w = q1

can = MultiDOFJointTrajectoryPoint()
can.transforms.append(Transform())
can.velocities.append(Twist())
x, y, z = 466.97, 205.71, 650.0
q1, q2, q3, q4 = 0.0, -0.26958, 0.96298, 0.0

can.transforms[0].translation.x = x
can.transforms[0].translation.y = y
can.transforms[0].translation.z = z
can.transforms[0].rotation.x = q2
can.transforms[0].rotation.y = q3
can.transforms[0].rotation.x = q4
can.transforms[0].rotation.w = q1


# get the current robot pose
robot_feedback = MultiDOFJointTrajectoryPoint()
robot_feedback = rospy.wait_for_message("/robot_feedback", MultiDOFJointTrajectoryPoint)
print (robot_feedback)

option = input()
if option == 1:
    goal = can
elif option ==2:
    goal = end_incline
else:
    goal = start_incline

t_arr, x_arr = interpolate(robot_feedback.transforms[0].translation.x, goal.transforms[0].translation.x)
plt.plot(t_arr, x_arr)
#plt.show()
t_arr, y_arr = interpolate(robot_feedback.transforms[0].translation.y, goal.transforms[0].translation.y)
plt.plot(t_arr, y_arr)
#plt.show()
t_arr, z_arr = interpolate(robot_feedback.transforms[0].translation.z, goal.transforms[0].translation.z)
plt.plot(t_arr, z_arr)
#plt.show()


robot_setpoints = robot_feedback
print t_arr.shape


for i in range(x_arr.shape[0]):
    robot_setpoints.transforms[0].translation.x = x_arr[i]
    robot_setpoints.transforms[0].translation.y = y_arr[i]
    robot_setpoints.transforms[0].translation.z = z_arr[i]
    robot_setpoints.transforms[0].rotation.x = q2
    robot_setpoints.transforms[0].rotation.y = q3
    robot_setpoints.transforms[0].rotation.z = q4
    robot_setpoints.transforms[0].rotation.w = q1
    pub.publish(robot_setpoints)
    rospy.sleep(egm_time_step)


# y_start = robot_set_point.transforms[0].translation.y
# y_goal = -500.0 # mm
# v_max = 2000.0
# t_final = abs(y_goal - y_start) / v_max

# num_steps = int((t_final)/egm_time_step)
# y_arr = np.linspace(y_start, y_goal, num_steps)
# t_arr = np.linspace(0, t_final, num_steps)
# # plt.plot(t_arr, y_arr)
# # plt.show()
