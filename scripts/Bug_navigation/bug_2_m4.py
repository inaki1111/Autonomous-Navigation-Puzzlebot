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


		############ ROBOT CONSTANTS ###############

		self.r = 0.05  # wheel radius [m]

		self.L = 0.19  # wheel separation [m]

		############ VARIABLES ###############

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

		self.distance_to_imaginary_line = 0.1 # distancia respecto a posicion actual del robot con linea imaginaria ax+by+c=0

		# inicializamos y definimos ganancias para goal to goal 
		self.k_v = 0.1
		self.k_w = 0.8

		#definimos minimo progreso requerido para ir a goal to goal
		self.minprogress = 1.0


		###******* INIT PUBLISHERS *******###

		self.pub_cmd_vel = rospy.Publisher('puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)  
		
		############################### SUBSCRIBERS #####################################
		
		rospy.Subscriber("puzzlebot_1/wl", Float32, self.wl_cb)  
		
		rospy.Subscriber("puzzlebot_1/wr", Float32, self.wr_cb)
		
		rospy.Subscriber("odom", Odometry, self.update_pos_cb) 
		
		rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb)

		#********** FRECUENCIA Y SAMPLING TIME **********###

		freq = 20

		rate = rospy.Rate(freq)  # freq Hz

		while rospy.get_time() == 0:
			print('NO simulated time')
		print('i got a valid time')

		self.a, self.b, self.c = self.calculate_line_params(2, 2, self.x_target, self.y_target)

		################ MAIN LOOP ################ 

		while not rospy.is_shutdown():           
			
			if self.lidar_received:
				
				######################## PRIMER ESTADO
				if self.current_state == 'Stop':

					if self.goal_received:

						print("Change to Go to goal from stop")

						self.current_state = "GoToGoal" 

						self.goal_received = 0
					else:

						self.v_msg.linear.x = 0.0

						self.v_msg.angular.z = 0.0

				######################## SEGUNDO ESTADO
				elif self.current_state == 'GoToGoal':
					
					# CALCULAR VEL Y THETA PARA IR AL GOAL. CALCULAR E_THETA PARA REALIZAR COMPARACION
					
					v, w, e_theta = self.move_to_goal()
					
					# ASIGNAR VEL Y THETA PARA IR AL GOAL


					
					self.v_msg.linear.x = v
					self.v_msg.angular.z = w
					
					
					# CALCULAR ANGULO DE THETAFWC PARA LUEGO REALIZAR COMPARACION DE GIRO EN FW

					wfwc = self.calculate_thetafwc()

					# PRIMERA COMPARACION EN GO TO GOAL ----
					# GIRA EN CLOCKWISE O HACIA MANECILLAS DEL RELOJ

					if self.closest_range < self.d_fw and abs(wfwc - e_theta) <= np.pi/2:
						
						# CALCULAR DISTANCIA ANTERIOR, h1
						
						self.dGTG_h = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)
						print("distancia anterior de GoalToGoal en FWC" + str(self.dGTG_h))
						
						
						# CAMBIAMOS DE INMEDIATO DE ESTADO
						self.current_state = 'FollowingWallClockwise'

					# SEGUNDA COMPARACION EN GO TO GOAL ----
					# GIRA EN COUNTER CLOCKWISE O HACIA MANECILLAS CONTRARIAS DEL RELOJ

					elif self.closest_range < self.d_fw and abs(wfwc - e_theta) > np.pi/2:
						
						# CALCULAR DISTANCIA ANTERIOR, h1

						self.dGTG_h = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)
						print("distancia anterior de GoalToGoal en FCC" + str(self.dGTG_h))
						
						# CAMBIAMOS DE INMEDIATO DE ESTADO
						self.current_state = 'FollowingWallCounterClockwise'
					
					# TERCERA COMPARACION EN GO TO GOAL ----
					# DETENER EL ROBOT

					elif np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) < self.target_position_tolerance:
						print(" DDISTANCIA AL GOAL ES MENOR AL TARGEEET: " + str(self.target_position_tolerance))
						self.current_state = 'Stop'

				######################## TERCER ESTADO
				elif self.current_state == 'FollowingWallClockwise':

					# CALCULAR VELOCIDAD LINEAL Y ANGULAR PARA LUEGO REALIZAR COMPARACIONES

					vfw, wfw = self.following_wallc() 

					self.v_msg.linear.x = vfw
					self.v_msg.angular.z = wfw
					
					# PRIMERA COMPARACION AL ESTAR RODEANDO LA PARED ----
					# COMPARAR LINEA AX+BY+C = 0 ESTA CERCA DE LA POSICION ACTUAL DEL ROBOT
					if self.clear_shot():
						
						self.current_state = 'GoToGoal'
					
					# SEGUNDA COMPARACION AL ESTAR RODEANDO LA PARED ----
					# DETENER EL ROBOT

					elif np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) < self.target_position_tolerance:
						print(" DDISTANCIA AL GOAL ES MENOR AL TARGEEET: " + str(self.target_position_tolerance))
						self.current_state = 'Stop'
				
				######################## TERCER ESTADO
				elif self.current_state == "FollowingWallCounterClockwise":

					# CALCULAR VELOCIDAD LINEAL Y ANGULAR PARA LUEGO REALIZAR COMPARACIONES

					vfw, wfw = self.following_wallcc()
					
					self.v_msg.linear.x = vfw
					self.v_msg.angular.z = wfw
					
					# PRIMERA COMPARACION AL ESTAR RODEANDO LA PARED ----
					# COMPARAR LINEA AX+BY+C = 0 ESTA CERCA DE LA POSICION ACTUAL DEL ROBOT
					if self.clear_shot():
						self.current_state = 'GoToGoal'

					# SEGUNDA COMPARACION AL ESTAR RODEANDO LA PARED ----
					# DETENER EL ROBOT

		
					elif np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) < self.target_position_tolerance:
						print(" DDISTANCIA AL GOAL ES MENOR AL TARGEEET: " + str(self.target_position_tolerance))
						self.current_state = 'Stop'

			self.pub_cmd_vel.publish(self.v_msg) # PUBLICAMOS VELOCIDADES LINEALES Y ANGULARES AL ROBOT
			print(self.current_state) # IMPRIMIMOS ESTADO ACTUAL
			rate.sleep()

	def update_pos_cb(self, odom):
		self.x_r = odom.pose.pose.position.x # POSICION EN X
		self.y_r = odom.pose.pose.position.y # POSICION EN Y
		
		# OBTENER QUATERNION
		orientation_q = odom.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		
		# CONVERTIR QUATERNION A ANGULO DE EULER
		_, _, self.theta_r = euler_from_quaternion(orientation_list)

	def move_to_goal(self): # FUNCION PARA CALCULAR VELOCIDADES LINEALES Y ANGULARES AL GOAL
		d = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) # CALCULAR DISTANCIA AL GOAL
		theta_G = np.arctan2(self.y_target - self.y_r, self.x_target - self.x_r) # CALCULAR ANGULO THETA G
		
		e_theta = theta_G - self.theta_r # CALCULAR ANGULO E_THETA
		e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) # LIMITAR ANGULO DE -PI A PI
		
		v = self.k_v * d
		w = self.k_w * e_theta

		print("distancia actual " + str(d) + "target tolerance " + str(self.target_position_tolerance))
		
		return v, w, e_theta

	def calculate_thetafwc(self): #FUNCION PARA CALCULAR ANGULO THETAFWC PARA PODER REALIZAR COMPARACION DE GO TO GOAL A FOLLOW WALL
		theta_AO = self.closest_angle + np.pi
		theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO)) #ARCTAN
		wfwc = -(np.pi / 2) + theta_AO
		wfwc = np.arctan2(np.sin(wfwc), np.cos(wfwc))
		return wfwc

	def following_wallc(self): # FUNCION PARA FOLLOW WALL EN SENTIDO DE MANECILLAS DEL RELOJ
		
		theta_AO = self.closest_angle + np.pi
		theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO)) # ARCTAN
		wfwc = -(np.pi / 2) + theta_AO
		wfwc = np.arctan2(np.sin(wfwc), np.cos(wfwc))
		
		vfw = 0.15 # CONSTANTE DE VELOCIDAD LINEAL
		kw = 1.5 # GANANCIA PARA GIRO EN W
		wfw = kw * wfwc
		return vfw, wfw

	def following_wallcc(self): # FUNCION PARA FOLLOW WALL EN SENTIDO DE MANECILLAS DEL RELOJ
		theta_AO = self.closest_angle + np.pi
		theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO)) # ARCTAN
		wfwcc = np.pi / 2 + theta_AO
		wfwcc = np.arctan2(np.sin(wfwcc), np.cos(wfwcc))
		
		vfw = 0.15 # CONSTANTE DE VELOCIDAD LINEAL
		kw = 1.5 # GANANCIA PARA GIRO EN W
		wfw = kw * wfwcc
		return vfw, wfw

	def calculate_line_params(self, x1, y1, x2, y2): # FUNCION PARA CALCULAR PARAMETROS DE LA ECUACION DISTANCIA ENTRE PUNTO Y LINEA
		a = y2 - y1
		b = x1 - x2
		c = x2 * y1 - x1 * y2
		return a, b, c

	def clear_shot(self): # FUNCION PARA VERIFICAR POSICION ACTUAL DEL ROBOT RESPECTO A LA LINEA AX+BY+C
		current_distance = abs(self.a * self.x_r + self.b * self.y_r + self.c) / np.sqrt(self.a ** 2 + self.b ** 2)
		d_GTG = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2)
		
		print("current " + str(current_distance) + " actual " + str(d_GTG) + "anterior dis " +str(self.dGTG_h))
		return current_distance < self.distance_to_imaginary_line and d_GTG < abs(self.dGTG_h - self.minprogress) # COMPRAR QUE LA DISTANCIA ENTRE LA LINEA Y POSICION ACTUAL DEL ROBOT


	def laser_cb(self, msg):
		self.closest_range = min(msg.ranges)
		idx = msg.ranges.index(self.closest_range)
		self.closest_angle = msg.angle_min + idx * msg.angle_increment
		self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle))
		self.lidar_received = 1

	def wl_cb(self, wl):

		## This function receives a the left wheel speed [rad/s] 
		self.wl = wl.data

	def wr_cb(self, wr):

		## This function receives a the right wheel speed.

		self.wr = wr.data

	def cleanup(self):

		#This function is called just before finishing the node  

		# You can use it to clean things up before leaving  

		# Example: stop the robot before finishing a node.

		vel_msg = Twist()

		self.pub_cmd_vel.publish(vel_msg)

############################### MAIN PROGRAM #################################### 

if __name__ == "__main__":

	rospy.init_node("bug_0", anonymous=True)

	AutonomousNav()
