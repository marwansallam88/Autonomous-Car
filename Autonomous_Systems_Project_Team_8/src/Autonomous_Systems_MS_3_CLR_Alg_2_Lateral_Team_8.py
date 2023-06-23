#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import time

rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_8.py', anonymous = True)

pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size = 10)
rate = rospy.Rate(10)

des_vel_msg = Twist()

#Set desired pose
x_des = 5
y_des = 3
v_des = 1
yaw_des = 0.0
L = 0.5
v = 1
i = 0

#Initial position
flag_initial_Pos = 0
xpos_0 = 0
ypos_0 = 0
roll_0 = 0
pitch_0 = 0
yaw_0 = 0
vel_0 = 0

#Current position
xpos = 0
ypos = 0
roll = 0
pitch = 0
yaw = 0
vel = 0

#Initialization of control gain
Kp = 2
acc = 0
v_new = 0

Total_time = 100.00 #total time of simulation
Ts = 0.01 #sampling time
Time = np.arange(0,Total_time + Ts,Ts)  #define the array of time steps
Num_pnts = len(Time)  #number of points in time array
v_d_vals = [0] * Num_pnts  #initialize an empty array
steer_vals = [0] * Num_pnts  #initialize an empty arrayi= 

#Define X/Y/Theta vals
x_arr = [0] * 1000
y_arr = [0] * 1000
yaw_arr = [0] * 1000
vel_arr = [0] * 1000
xerr_arr = [0] * 1000
yerr_arr = [0] * 1000
yaw_err_arr = [0] * 1000
vel_err_arr = [0] * 1000
d = [0, 0, 0, 0, 0, 0, 0, 0, 0] * 100

time.sleep(10)

def callback_Init(data):
	global sub0, xpos_0, ypos_0, vel_0, roll_0, pitch_0, yaw_0, flag_initial_Pos
	flag_initial_Pos = 1
	orientation_q = data.pose[1].orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll_0, pitch_0, yaw_0) = euler_from_quaternion (orientation_list)
	xpos_0 = round(data.pose[1].position.x,4)
	ypos_0 = round(data.pose[1].position.y,4)	
	vel_0 = round(data.twist[1].linear.x,4)
	sub0.unregister()

sub0 = rospy.Subscriber("/steer_bot/gazebo/model_states", ModelStates, callback_Init)

def callback(data):
	global sub1, xpos, ypos, vel, roll, pitch, yaw, i
	orientation_q = data.pose[1].orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	xpos = round(data.pose[1].position.x,4)
	ypos = round(data.pose[1].position.y,4)	
	vel = round(data.twist[1].linear.x,4)

    
sub1 = rospy.Subscriber("/steer_bot/gazebo/model_states", ModelStates, callback)

def desiredSteeringAngle():
	global phi, v
	ld = np.sqrt(np.square(x_des-xpos)+np.square(y_des-ypos))
	alpha = (- yaw + np.arctan((y_des - ypos) / (1)))
	phi = np.arctan(2 * L * np.sin(alpha) / ld)

def desiredVeloctiy(v_act, Ts):
	global v_new, acc, kp
	acc = Kp * (v_des - vel)

	if (np.abs(acc) > Kp):
		acc = 2 * np.sign(acc)
		 
	v_new = Ts * acc + v_act
	print("Vnew ="+ str(v_new))
		
#Stop code here till subscribe the first msg of the vehicle position
while flag_initial_Pos == 0:
	pass
	
for t in Time:

	st_time = time.time()

	print("Xpos:" + str(xpos) + "	 Ypos:" + str(ypos) + "	 Theta:" + str(yaw)+"	 Velocity:" + str(vel) + "Xerr:" + str(x_des - xpos) + "	 Yerr:" + str(y_des - ypos) + "	 yaw_err_arr:" + str(yaw_des - yaw) + "	 Vel_err:" + str(v_des - vel) + "  Time:" + str(Time[i]))
	
	x_arr[i] = xpos
	y_arr[i] = ypos
	yaw_arr[i] = yaw
	vel_arr[i] = vel
	xerr_arr[i] = x_des - xpos
	yerr_arr[i] = y_des - ypos
	yaw_err_arr[i] = yaw_des - yaw
	vel_err_arr[i] = v_des - vel
	
	v_act = vel
	desiredVeloctiy(v_act, Ts)
	v = v_new
	
	if abs(x_des - xpos) <= 0.5 and abs(y_des - ypos) <= 0.5:
		v = v / 3
		
	if abs(x_des - xpos) <= 0.2 and abs(y_des - ypos) <= 0.2:
		v = 0

	desiredSteeringAngle()
	w = (v / L) * np.tan(phi)
	
	des_vel_msg.linear.x = v
	des_vel_msg.linear.y = 0
	des_vel_msg.linear.z = 0
	des_vel_msg.angular.x = 0
	des_vel_msg.angular.y = 0
	des_vel_msg.angular.z = w 

	i +=1
	
	np.savetxt('X8.csv', x_arr, newline='\n')
	np.savetxt('Y8.csv', y_arr, newline='\n')
	np.savetxt('THETA8.csv', yaw_arr, newline='\n')
	np.savetxt('VEL8.csv', vel_arr, newline='\n')
	
	np.savetxt('XERR8.csv', xerr_arr, newline='\n')
	np.savetxt('YERR8.csv', yerr_arr, newline='\n')
	np.savetxt('THETAERR8.csv', yaw_err_arr, newline='\n')
	np.savetxt('VELERR8.csv', vel_err_arr, newline='\n')
	
	np.savetxt('TIME8.csv', Time, newline='\n')
	
	end_time = time.time()
	Ts  = end_time-st_time
	if not rospy.is_shutdown():
		pub.publish(des_vel_msg)	
		rate.sleep()


