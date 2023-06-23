#!/usr/bin/env python

import rospy
import time
import math

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sympy import *

import numpy as np
import sympy as sym

time.sleep(10)

rospy.init_node('Autonomous_Systems_MS_4_Path_Planning_Team_8.py', anonymous=False)
rate = rospy.Rate(10)

pub1 = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size=10) 

Des_Vel_Msg = Twist() #Initializing Desired Velocity Message 

L = 0.5 #Wheelbase Length
Ts = 0.1 #Sampling Time
theta_des = 0.0 #Desired Orientation
v_des = 0.5 #Desired Velocity

#Initial position
flag_initial = 0
pos_0 = np.zeros((1,3))
vel_0 = np.zeros((1,3))

def callback_Init(data):
	global sub0, flag_initial, pos_0, vel_0
	x_pos_0 = round(data.pose.pose.position.x, 4)
	y_pos_0 = round(data.pose.pose.position.y, 4)
	z_pos_0 = round(data.pose.pose.position.z, 4)
	x_ori_0 = round(data.pose.pose.orientation.x, 4)
	y_ori_0 = round(data.pose.pose.orientation.y, 4)
	z_ori_0 = round(data.pose.pose.orientation.z, 4)
	w_ori_0 = round(data.pose.pose.orientation.w, 4)
	
	orientation = [x_ori_0, y_ori_0, z_ori_0, w_ori_0]
	(roll, pitch, yaw) = euler_from_quaternion(orientation)
	pos = [x_pos_0, y_pos_0, yaw]	
	print(pos[0])
	
	#Creating Current Velocity List
	x_vel_0 = round(data.twist.twist.linear.x, 4)
	y_vel_0 = round(data.twist.twist.linear.y, 4)
	z_vel_0 = round(data.twist.twist.linear.z, 4)
	x_omega_0 = round(data.twist.twist.angular.x, 4)
	y_omega_0 = round(data.twist.twist.angular.y, 4)
	z_omega_0 = round(data.twist.twist.angular.z, 4)
	
	vel = [x_vel_0, z_omega_0]
	
	flag_initial = 1
	sub0.unregister()

sub0 = rospy.Subscriber('/steer_bot/ackermann_steering_controller/odom', Odometry, callback_Init)

#Define the initial pose and velocity of the vehicle
rob_pos_0 = [pos_0[0,0], pos_0[0, 1], pos_0[0, 2]]
rob_vel_0 = [vel_0[0, 0],vel_0[0, 1]]

x_pos = rob_pos_0[0]
y_pos = rob_pos_0[1]

x_vel = rob_vel_0[0] * np.cos(rob_pos_0[2])
y_vel = rob_vel_0[0] * np.sin(rob_pos_0[2])

#Current position
flag_cont = 0
pos = np.zeros((1,3))
vel = np.zeros((1,3))

def callback(data):
	global sub1, flag_cont, pos, vel
	#Creating Current Position List
	x_pos = round(data.pose.pose.position.x, 4)
	y_pos = round(data.pose.pose.position.y, 4)
	z_pos = round(data.pose.pose.position.z, 4)
	x_ori = round(data.pose.pose.orientation.x, 4)
	y_ori = round(data.pose.pose.orientation.y, 4)
	z_ori = round(data.pose.pose.orientation.z, 4)
	w_ori = round(data.pose.pose.orientation.w, 4)
	
	orientation = [x_ori, y_ori, z_ori, w_ori]
	(roll, pitch, yaw) = euler_from_quaternion(orientation)
	pos = [x_pos, y_pos, yaw]	
	
	#Creating Current Velocity List
	x_vel = round(data.twist.twist.linear.x, 4)
	y_vel = round(data.twist.twist.linear.y, 4)
	z_vel = round(data.twist.twist.linear.z, 4)
	x_omega = round(data.twist.twist.angular.x, 4)
	y_omega = round(data.twist.twist.angular.y, 4)
	z_omega = round(data.twist.twist.angular.z, 4)
	
	vel = [x_vel, z_omega]
	
	flag_cont = 1

sub1 = rospy.Subscriber('/steer_bot/ackermann_steering_controller/odom', Odometry, callback)

#Stop code here till subscribe the first msg of the vehicle position
while flag_initial == 0:
	pass

def Pure_Pursuit_Control(pos_des, pos_act):
	global phi, v
	x_des = pos_des[0]
	y_des = pos_des[1]
	x_pos = pos_act[0]
	y_pos = pos_act[1]
	yaw = pos_act[2]
	ld = np.sqrt(np.square(x_des-x_pos)+np.square(y_des-y_pos))
	alpha = (- yaw + np.arctan((y_des - y_pos) / (1)))
	phi = np.arctan(2 * L * np.sin(alpha) / ld)
	return(phi)

def desiredVeloctiy(v_des, v_act, Ts):
	Kp = 2
	acc = Kp * (v_des - v_act)

	if (np.abs(acc) > Kp):
		acc = 2 * np.sign(acc)
		 
	v_new = Ts * acc + v_act
	return(v_new)

while 1 and not rospy.is_shutdown():	
	
	if pos[0] <= 3:
		pos_des = [3, -0.8]
		
	elif pos[0] > 3 and pos[0] <= 5:
		pos_des = [5, -0.8]
		
	elif pos[0] > 5  and pos[0] <= 7.5:
		pos_des = [7.5, 0]

	elif pos[0] > 7.5:
		pos_des = [10, 0]
	
	v = desiredVeloctiy(0.5, vel[0], Ts)
	
	if np.abs(pos[0] - 10) <= 0.2:
		v = 0
			
	phi = Pure_Pursuit_Control(pos_des, pos)
	w = (v / L) * np.tan(phi)
	
	Des_Vel_Msg.linear.x = v
	Des_Vel_Msg.linear.y = 0
	Des_Vel_Msg.linear.z = 0
	Des_Vel_Msg.angular.x = 0
	Des_Vel_Msg.angular.y = 0
	Des_Vel_Msg.angular.z = w 

	pub1.publish(Des_Vel_Msg)
	rate.sleep()		#Sleep with rate
#########################################################################################################
