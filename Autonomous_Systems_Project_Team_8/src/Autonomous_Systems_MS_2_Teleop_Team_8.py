#!/usr/bin/env python

from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
from select import select

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

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
b : stop
w/s : increase/decrease only linear speed
a/d : increase/decrease only angular speed  Left/Right

CTRL-C to quit
"""

moveBindings = {
        'b':(0,0,0,0),
    }

speedBindings={
        'w':(1,0),
        's':(-1,0),
        'a':(0,1),
        'd':(0,-1),
        'b':(0,0)
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/steer_bot/ackermann_steering_controller/cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(1, 0, 0, 1, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            w = (self.speed/l)*np.tan(self.turn*(np.pi/180))
            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * w
            

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)
    
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



if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('Autonomous_Systems_MS_2_Teleop_Team_8.py')

    speed = rospy.get_param("~speed", 0.0)
    turn = rospy.get_param("~turn", 0.0)
    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 180)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)

    x = 1
    y = 0
    z = 0
    th = 1
    speed = 0
    turn = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                speed = moveBindings[key][0]
                turn = moveBindings[key][1]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed + speedBindings[key][0])
                turn = min(turn_limit, turn + speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)#!/usr/bin/env python
