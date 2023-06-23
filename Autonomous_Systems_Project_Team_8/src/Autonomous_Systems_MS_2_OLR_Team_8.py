#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

rospy.init_node('Autonomous_Systems_MS_2_OLR_Team_8.py', anonymous = True)

Pub = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', Twist, queue_size = 10)

vel_msg = Twist() 
rate = rospy.Rate(10)
flag_initial_Pos = 0

#Initial position
xpos_0 = 0
ypos_0 = 0
roll_0 = 0
pitch_0 = 0
yaw_0 = 0
vel_0 = 0
######
xpos = 0
ypos = 0
roll = 0
pitch = 0
yaw = 0
vel = 0
l = 0.1
Total_time = 100.00
Ts = 0.1

Time = np.arange(0, Total_time + Ts, Ts)
driving_velocity_vals = [3] * len(Time)  
steering_angle_vals = [-35] * len(Time) 

def callback_Init(data):
	global sub1, xpos_0, ypos_0, vel_0, roll_0, pitch_0, yaw_0, flag_initial_Pos
	flag_initial_Pos = 1
	orientation_q = data.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll_0, pitch_0, yaw_0) = euler_from_quaternion (orientation_list)
	xpos_0 = round(data.pose.pose.position.x,4)
	ypos_0 = round(data.pose.pose.position.y,4)	
	vel_0 = round(data.twist.twist.linear.x,4)
	print("Xpos_0:" + str(xpos_0) + "	 Ypos_0:" + str(ypos_0) + "		Roll_0:" + str(roll_0) + "		Pitch_0:" + str(pitch_0) + "		Yaw_0:" + str(yaw_0) + "		Velocity_0:" + str(vel_0))
	sub1.unregister()

sub1 = rospy.Subscriber('/steer_bot/ackermann_steering_controller/odom', Odometry, callback_Init)

def callback(data):
    global sub2, xpos, ypos, vel, roll, pitch, yaw
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    xpos = round(data.pose.pose.position.x,4)
    ypos = round(data.pose.pose.position.y,4)	
    vel = round(data.twist.twist.linear.x,4)
    print("Xpos:" + str(xpos) + "	 Ypos:" + str(ypos) + "		Roll:" + str(roll) + "		Pitch:" + str(pitch) + "		Yaw:" + str(yaw) + "		Velocity:" + str(vel))
    
sub2 = rospy.Subscriber('/steer_bot/ackermann_steering_controller/odom', Odometry, callback)

##Stop code here till subscribe the first msg of the vehicle position
while flag_initial_Pos == 0:
	pass
	
i = 0 
for t in Time:
   
    v = driving_velocity_vals[i] 
    steering_angle = steering_angle_vals[i]*(np.pi/180) 
    w = (v/l)*np.tan(steering_angle)
	
    vel_msg.linear.x = v
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = w
    
    if not rospy.is_shutdown():
        Pub.publish(vel_msg)	
        rate.sleep()	

    i +=1
