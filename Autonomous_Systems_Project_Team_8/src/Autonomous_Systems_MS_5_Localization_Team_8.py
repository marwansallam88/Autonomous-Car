#! /usr/bin/env python 

# Libraries to be imported
import time
import math
import rospy 
import random
import numpy as np
import sympy as sym
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

# Initialize ROS Node 
rospy.init_node('Autonomous_Systems_MS_5_Localization_Team_8.py', anonymous = True)

# ROS Publisher Code
pub = rospy.Publisher('/filtered_values', Odometry, queue_size = 10) 

# Rate of publishing msg 10hz
rate = rospy.Rate(10) 

# Filtered States Message:
filtered_states = Odometry()

# Create arrays for printing:
# Array for Time:
TimeTotal = []

# Arrays for States:
XModel = [] 
YModel = []
ThetaModel = [] 

# Arrays for Noisy Signals:
XNoisy = [] 
YNoisy = [] 
ThetaNoisy = [] 

# Arrays for Filtered Signals:
XFiltered = []
YFiltered = []
ThetaFiltered = []

# Create an array to have the noisy measurments
position_noisy = np.zeros(6)
count = 0

# Create method to add noise to the sensor
def noisy_add(position):
	#Define the global variables represented as the standard deviation of each state
	global position_noisy, XNoisy, YNoisy, ZNoisy, ThetaNoisy
	
	xnoise = position[0] + random.gauss(0,0.05)			
	ynoise = position[1] + random.gauss(0,0.05)			
	thetanoise = position[5] + random.gauss(0,0.05)	

	position_noisy = [xnoise, ynoise, position[2], position[3], position[4], thetanoise]
	return position_noisy

# Initialization:
P = [[100,0,0],[0,100,0],[0,0,1]]		
Q = [[0.01,0,0],[0,0.01,0],[0,0,0.01]]		
R = 0.1						

U = [[0.2],[0]]					
X = [[0],[0],[0]]				

A = [[1,0,0],[0,1,0],[0,0,1]]			
B = [[0.1,0],[0,0],[0,0.1]]			
C = [[1,0,0]]					

# Prediction stage in Kalman filter
def kf_prediction(Xprev, Pprev, A, Q, B, U):
	Xpredicted = np.matmul(A, Xprev) + np.dot(B, U)						
	Ppredicted = np.matmul(A, np.matmul(Pprev, np.transpose(A))) + Q	
	return (Xpredicted, Ppredicted) 
    
# Correction stage in Kalman filter
def kf_correction(Xpredicted, Ppredicted, C, Z, R):			
	CTrans = np.transpose(C)				
	num = np.matmul(Ppredicted, CTrans)		
	den1 = np.matmul(C, Ppredicted) 
	den2 = np.matmul(den1, CTrans) + R  
	den = np.matrix(den2)  			
	deninv = den.getI()					
	KGain = np.matmul(num, deninv) 

	Xfiltered = Xpredicted + np.matmul(KGain, (Z - np.matmul(C, Xpredicted))) 	
	Pfiltered = Ppredicted - np.matmul(KGain, np.matmul(C, Ppredicted)) 	
	return (Xfiltered, Pfiltered)
	
 
# ROS Subscriber Code for Position
flag_cont = 0			
position = np.zeros(6)	
velocity = np.zeros(6)
Time = 0

# Callback function for feedback the vehicle current position
def callback(data):
	global sub1, flag_cont, position, velocity, Time

	x_pos =  round(data.pose[5].position.x, 4)
	y_pos =  round(data.pose[5].position.y, 4)
	z_pos =  round(data.pose[5].position.z, 4)
	x_ori =  round(data.pose[5].orientation.x, 4)
	y_ori =  round(data.pose[5].orientation.y, 4)
	z_ori =  round(data.pose[5].orientation.z, 4)
	w_ori =  round(data.pose[5].orientation.w, 4)
	[roll, pitch, yaw] = euler_from_quaternion([x_ori, y_ori, z_ori, w_ori])
	 
	position = [x_pos, y_pos, z_pos, roll, pitch, yaw] 
	print(position)
	
	vel_x = round(data.twist[5].linear.x, 4)
	vel_y = round(data.twist[5].linear.y, 4)
	vel_z = round(data.twist[5].linear.z, 4)
	w_x = round(data.twist[5].angular.x, 4)
	w_y = round(data.twist[5].angular.y, 4)
	w_z = round(data.twist[5].angular.z, 4)

	velocity = [vel_x, vel_y, vel_z, w_x, w_y, w_z]
	
	Time = time.time() - Time
	flag_cont = 1				

sub1 = rospy.Subscriber("/steer_bot/gazebo/model_states", ModelStates, callback) 

# ROS Subscriber Code for Initial Position
flag_initial = 0	      
velocity_0 = np.zeros(6)  
position_0 = np.zeros(6)  
Time_0 = 0

# Initial callback function for setting the vehicle initial position
def callback_Init(data): 
	global sub0, flag_initial, position_0, velocity_0, Time_0

	x_pos_0 =  round(data.pose[5].position.x, 4)
	y_pos_0 =  round(data.pose[5].position.y, 4)
	z_pos_0 =  round(data.pose[5].position.z, 4)
	x_ori_0 =  round(data.pose[5].orientation.x, 4)
	y_ori_0 =  round(data.pose[5].orientation.y, 4)
	z_ori_0 =  round(data.pose[5].orientation.z, 4)
	w_ori_0 =  round(data.pose[5].orientation.w, 4)
	[roll_0, pitch_0, yaw_0] = euler_from_quaternion([x_ori_0, y_ori_0, z_ori_0, w_ori_0]) 
	
	position_0 = [x_pos_0, y_pos_0, z_pos_0, roll_0, pitch_0, yaw_0] 
	print(position_0)
	
	vel_x_0 = round(data.twist[5].linear.x, 4)
	vel_y_0 = round(data.twist[5].linear.y, 4)
	vel_z_0 = round(data.twist[5].linear.z, 4)
	w_x_0 = round(data.twist[5].angular.x, 4)
	w_y_0 = round(data.twist[5].angular.y, 4)
	w_z_0 = round(data.twist[5].angular.z, 4)
	
	velocity_0 = [vel_x_0, vel_y_0, vel_z_0, w_x_0, w_y_0, w_z_0]
	
	Time_0 = time.time() - Time_0
	flag_initial = 1
	sub0.unregister()			
	
sub0 = rospy.Subscriber("/steer_bot/gazebo/model_states", ModelStates, callback_Init) 

# Stop code here till subscribe the first msg of the vehicle position
while flag_initial == 0:
	pass

curr_vel = np.zeros(6)
iterations = 1000
i = 0

while 1 and not rospy.is_shutdown():
	if i < iterations:
		if flag_initial == 1:
			curr_vel = velocity_0
			position_noisy = noisy_add(position_0)
			x_noise = position_0[0]
			y_noise = position_0[1]
			theta_noise = position_0[5]
			t = Time_0
			
			X = [[x_noise],[y_noise],[theta_noise]]
			Z = [x_noise, y_noise, theta_noise]
			
			flag_initial = 0
			
		elif flag_cont == 1:
			curr_vel = velocity
			position_noisy = noisy_add(position)
			x_noise = position_noisy[0]
			y_noise = position_noisy[1]
			theta_noise = position_noisy[5]	
			t = Time 
			
			Z = [x_noise, y_noise, theta_noise]
			X = Xfiltered
			P = Pfiltered
			
			flag_cont = 0
					
		(Xpredicted, Ppredicted) = kf_prediction(X, P, A, Q, B, U)
		(Xfiltered, Pfiltered) = kf_correction(Xpredicted, Ppredicted, C, Z , R)
		
		XNoisy.append(x_noise) 
		YNoisy.append(y_noise)	
		ThetaNoisy.append(theta_noise)
		
		x_model = Xpredicted.item(0)
		y_model = Xpredicted.item(1)
		theta_model = Xpredicted.item(2)

		XModel.append(x_model) 	
		YModel.append(y_model) 
		ThetaModel.append(theta_model) 
				
		x_filtered = Xfiltered.item(0)
		y_filtered = Xfiltered.item(1)
		theta_filtered = Xfiltered.item(2)

		XFiltered.append(x_filtered) 
		YFiltered.append(y_filtered)	
		ThetaFiltered.append(theta_filtered) 
		
		TimeTotal.append(t)
		
		filtered_states.pose.pose.position.x = x_filtered
		filtered_states.pose.pose.position.y = y_filtered
		filtered_states.pose.pose.position.z = position_noisy[2]
		
		[x_ori, y_ori, z_ori, w_ori] = quaternion_from_euler(position_noisy[3], position_noisy[4], theta_filtered)
		
		filtered_states.pose.pose.orientation.x = x_ori
		filtered_states.pose.pose.orientation.y = y_ori
		filtered_states.pose.pose.orientation.z = z_ori
		filtered_states.pose.pose.orientation.w = w_ori
		
		filtered_states.twist.twist.linear.x = curr_vel[0]
		filtered_states.twist.twist.linear.y = curr_vel[1]
		filtered_states.twist.twist.linear.z = curr_vel[2]
		filtered_states.twist.twist.angular.x = curr_vel[3]
		filtered_states.twist.twist.angular.y = curr_vel[4]
		filtered_states.twist.twist.angular.z = curr_vel[5]
		
		pub.publish(filtered_states)
		rate.sleep()
	else:
		rospy.signal_shutdown("for plotting")
		
	i = i + 1
		
# Plotting of signals from sensor and noisy signals
fig1, (xmodel, xnoisy, xfiltered) = plt.subplots(3)
fig1.suptitle('Values of State "X"')
xmodel.plot(XModel, 'r-', label='X-Model')
xnoisy.plot(XNoisy, 'b-', label='X-Noisy')
xfiltered.plot(XFiltered, 'g-', label='X-Filtered')
plt.legend()

fig2, (ymodel, ynoisy, yfiltered) = plt.subplots(3)
fig2.suptitle('Values of State "Y"')
ymodel.plot(YModel, 'r-', label='Y-Model')
ynoisy.plot(YNoisy, 'b-', label='Y-Noisy')
yfiltered.plot(YFiltered, 'g-', label='Y-Filtered')
plt.legend()

fig3, (thetamodel, thetanoisy, thetafiltered) = plt.subplots(3)
fig3.suptitle('Values of State "Theta"')
thetamodel.plot(np.degrees(ThetaModel), 'r-', label='Theta-Model')
thetanoisy.plot(np.degrees(ThetaNoisy), 'b-', label='Theta_Noisy')
thetafiltered.plot(np.degrees(ThetaFiltered), 'g-', label='Theta-Filtered')
plt.legend()

plt.show(block=True)

