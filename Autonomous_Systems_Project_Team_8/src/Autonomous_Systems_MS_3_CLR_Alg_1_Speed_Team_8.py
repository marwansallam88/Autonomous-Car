#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

time.sleep(10)

rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_8.py', anonymous = True)

rate = rospy.Rate(10)

pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size = 10)

des_vel_msg = Twist()

v_des = 1

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
delta_t = 0.01
l = 0.1
steering_angle = 0
acc = 0
v_new = 0
st_time = time.time()

def callback_Init(data):
	global sub0, xpos_0, ypos_0, vel_0, roll_0, pitch_0, yaw_0, flag_initial_Pos
	flag_initial_Pos = 1
	orientation_q = data.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll_0, pitch_0, yaw_0) = euler_from_quaternion (orientation_list)
	xpos_0 = round(data.pose.pose.position.x,4)
	ypos_0 = round(data.pose.pose.position.y,4)	
	vel_0 = round(data.twist.twist.linear.x,4)
	print("Xpos_0:" + str(xpos_0) + "	 Ypos_0:" + str(ypos_0) + "		Roll_0:" + str(roll_0) + "		Pitch_0:" + str(pitch_0) + "		Yaw_0:" + str(yaw_0) + "		Velocity_0:" + str(vel_0))
	sub0.unregister()

sub0 = rospy.Subscriber('/steer_bot/ackermann_steering_controller/odom', Odometry, callback_Init)


def callback(data):
    global sub1, xpos, ypos, vel, roll, pitch, yaw
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    xpos = round(data.pose.pose.position.x,4)
    ypos = round(data.pose.pose.position.y,4)	
    vel = round(data.twist.twist.linear.x,4)
    print("Xpos:" + str(xpos) + "	 Ypos:" + str(ypos) + "		Roll:" + str(roll) + "		Pitch:" + str(pitch) + "		Yaw:" + str(yaw) + "		Velocity:" + str(vel))
    
sub1 = rospy.Subscriber('/steer_bot/ackermann_steering_controller/odom', Odometry, callback)

def desiredVeloctiy(v_act, delta_t):
	global v_new, acc, Kp
	acc = Kp * (v_des - v_act)
	
	if (np.abs(acc) > Kp):
		acc = 2 * np.sign(acc)
		 
	v_new = delta_t * acc + v_act
	print("Vnew ="+ str(v_new))
	
#Stop code here till subscribe the first msg of the vehicle position
while flag_initial_Pos == 0:
	pass

while 1 and not rospy.is_shutdown():
	st_time = time.time()
	v_act = vel
	desiredVeloctiy(v_act, delta_t)
	v = v_new
	w = (v_new/l)*np.tan(steering_angle)
	

	des_vel_msg.linear.x = v  
	des_vel_msg.linear.y = 0
	des_vel_msg.linear.z = 0
	des_vel_msg.angular.x = 0
	des_vel_msg.angular.y = 0
	des_vel_msg.angular.z = w 

	pub.publish(des_vel_msg)
		
	rate.sleep()	
	end_time = time.time()
	delta_t  = end_time-st_time
