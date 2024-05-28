#!/usr/bin/env python3  

import rospy  
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan   #Lidar
from tf.transformations import euler_from_quaternion
import numpy as np 

# This class will make the puzzlebot move to a given goal 
class AutonomousNav():  
	def __init__(self):  
		rospy.on_shutdown(self.cleanup) 

		self.r = 0.05  # wheel radius [m]
		self.L = 0.19  # wheel separation [m] 

		self.dGTG_h = 0
		self.closest_angle = 0
		self.closest_range = 0
		self.theta_r = 0
		self.x_r = 0
		self.y_r = 0

		self.x_target = 0.0  # x position of the goal 
		self.y_target = -2.3    # y position of the goal

		self.goal_received = 1  # flag to indicate if the goal has been received 
		self.lidar_received = 0  # flag to indicate if the laser scan has been received 
		self.target_position_tolerance = 0.10  # acceptable distance to the goal to declare the robot has arrived to it [m] 
		self.d_fw = 0.55  # distance from closest obstacle to activate the avoid obstacle behavior [m] 

		self.v_msg = Twist()  # Robot's desired speed  
		self.wr = 0  # right wheel speed [rad/s]
		self.wl = 0  # left wheel speed [rad/s]

		self.current_state = 'GoToGoal'  # Robot's current state 

		# Gains for move to goal control
		self.k_v = 0.1
		self.k_w = 0.8

		# Minimum progress required to change state
		self.minprogress = 1.0

		self.pub_cmd_vel = rospy.Publisher('puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)  
		rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb)  
		rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb)
		rospy.Subscriber("odom", Odometry, self.update_pos_cb) 
		rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb) 

		freq = 20 
		rate = rospy.Rate(freq)  # freq Hz

		while rospy.get_time() == 0:
			print('NO simulated time')
		print('i got a valid time')

		self.previous_time = rospy.get_time()

		self.a, self.b, self.c = self.calculate_line_params(2, 2, self.x_target, self.y_target)

		while not rospy.is_shutdown():           
			if self.lidar_received:
				if self.current_state == 'Stop':
					if self.goal_received:
						print("Change to Go to goal from stop") 
						self.current_state = "GoToGoal" 
						self.goal_received = 0
					else:
						self.v_msg.linear.x = 0.0 
						self.v_msg.angular.z = 0.0
				elif self.current_state == 'GoToGoal':
					v, w, e_theta = self.move_to_goal()
					self.v_msg.linear.x = v
					self.v_msg.angular.z = w
					wfwc = self.calculate_thetafwc()
					if self.closest_range < self.d_fw and abs(wfwc - e_theta) <= np.pi/2:
						self.dGTG_h = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)
						print("distancia anterior de GoalToGoal en FWC" + str(self.dGTG_h))
						self.current_state = 'FollowingWallClockwise'
					elif self.closest_range < self.d_fw and abs(wfwc - e_theta) > np.pi/2:
						self.dGTG_h = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)
						print("distancia anterior de GoalToGoal en FCC" + str(self.dGTG_h))
						self.current_state = 'FollowingWallCounterClockwise'
					elif np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) < self.target_position_tolerance:
						self.current_state = 'Stop'
				elif self.current_state == 'FollowingWallClockwise':
					vfw, wfw = self.following_wallc()
					self.v_msg.linear.x = vfw
					self.v_msg.angular.z = wfw
					if self.clear_shot():
						self.current_state = 'GoToGoal'
						#self.a, self.b, self.c = self.calculate_line_params(self.x_r, self.y_r, self.x_target, self.y_target)
					elif np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) < self.target_position_tolerance:
						self.current_state = 'Stop'
				elif self.current_state == "FollowingWallCounterClockwise":
					vfw, wfw = self.following_wallcc()
					self.v_msg.linear.x = vfw
					self.v_msg.angular.z = wfw
					if self.clear_shot():
						self.current_state = 'GoToGoal'
						#self.a, self.b, self.c = self.calculate_line_params(self.x_r, self.y_r, self.x_target, self.y_target)
					elif np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) < self.target_position_tolerance:
						self.current_state = 'Stop'

			self.pub_cmd_vel.publish(self.v_msg)
			print(self.current_state)
			rate.sleep()

	def update_pos_cb(self, odom):
		self.x_r = odom.pose.pose.position.x
		self.y_r = odom.pose.pose.position.y
		orientation_q = odom.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		_, _, self.theta_r = euler_from_quaternion(orientation_list)

	def move_to_goal(self):
		d = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)
		theta_G = np.arctan2(self.y_target - self.y_r, self.x_target - self.x_r)
		e_theta = theta_G - self.theta_r
		e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
		v = self.k_v * d
		w = self.k_w * e_theta
		return v, w, e_theta

	def calculate_thetafwc(self):
		theta_AO = self.closest_angle + np.pi
		theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO))
		wfwc = -(np.pi / 2) + theta_AO
		wfwc = np.arctan2(np.sin(wfwc), np.cos(wfwc))
		return wfwc

	def following_wallc(self):
		theta_AO = self.closest_angle + np.pi
		theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO))
		wfwc = -(np.pi / 2) + theta_AO
		wfwc = np.arctan2(np.sin(wfwc), np.cos(wfwc))
		vfw = 0.15
		kw = 1.5
		wfw = kw * wfwc
		return vfw, wfw

	def following_wallcc(self):
		theta_AO = self.closest_angle + np.pi
		theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO))
		wfwcc = np.pi / 2 + theta_AO
		wfwcc = np.arctan2(np.sin(wfwcc), np.cos(wfwcc))
		vfw = 0.15
		kw = 1.5
		wfw = kw * wfwcc
		return vfw, wfw

	def calculate_line_params(self, x1, y1, x2, y2):
		a = y2 - y1
		b = x1 - x2
		c = x2 * y1 - x1 * y2
		return a, b, c

	def clear_shot(self):
		current_distance = abs(self.a * self.x_r + self.b * self.y_r + self.c) / np.sqrt(self.a ** 2 + self.b ** 2)
		d_GTG = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)
		
		print("current " + str(current_distance) + " actual " + str(d_GTG) + "anterior dis " +str(self.dGTG_h))
		
		return current_distance < 0.1 and d_GTG < abs(self.dGTG_h - self.minprogress)

		#return current_distance < self.dGTG_h - self.minprogress

	def laser_cb(self, msg):
		self.closest_range = min(msg.ranges)
		idx = msg.ranges.index(self.closest_range)
		self.closest_angle = msg.angle_min + idx * msg.angle_increment
		self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle))
		self.lidar_received = 1

	def wl_cb(self, wl):
		self.wl = wl.data

	def wr_cb(self, wr):
		self.wr = wr.data

	def cleanup(self):
		vel_msg = Twist()
		self.pub_cmd_vel.publish(vel_msg)

if __name__ == "__main__":
	rospy.init_node("bug_0", anonymous=True)
	AutonomousNav()
