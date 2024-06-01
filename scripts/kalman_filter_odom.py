#!/usr/bin/env python

import rospy  
from std_msgs.msg import Float32, Float64, Int64, String
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np 

# Este programa predice y corrige segun el teorema de Kalman
# El primer paso a realizar es tomar las velocidades de las ruedas de nuestro
# robot para poder realizar un estimacion con error de localizacion con puntos finitos

# es decir, se predice una estimacion futura con el movimiento del robot 
# y la posicion anterior que este entrega. Amdemas, con esto podemos saber la covarianza o error
# que existe en nuestro sistema debido a las perturbaciones del ambiente donde
# se controla al robot.

# Es por ello, que con la parte de prediccion, que es realmente el filtro de Kalman
# donde participa, nos permite poder corregir con modelos de observacion el comportamiento
# en tiempo real que tiene nuestro robot ajustando su posicion cada vez que ve un Aruco.

# Por lo tanto, el filtro de Kalman siempre estara corrigiendo con su posicion, sin embargo
# se ajustara al modelo de observacion entregado por la camara con el Aruco, si no
# a la pequena variacion de la camara respecto a las posiciones anteriores desde donde
# inicio nuestro robot.

# This class will subscribe to the /wr and /wl topics and publish to /odom  

class Localisation():  
	def __init__(self):  
		# First, initialize a node
		rospy.init_node('localisation')

		self.current_miu = np.array([0,0,0])

		# VARIABLES A PARTIR DEL USO DE FILTRO DE BAYES CON PROPAGACION GAUSSIANA

		# variables para nuestro zp, k y ztheta, k
		self.delta_x = 0
		self.delta_y = 0

		self.zk = np.array([[0,0],[0,0]])
		self.theta_G = 0

		self.I = np.array([[1,0,0],[0,1,0],[0,0,1]]) # incertidumbre que se tiene
		
		# variables que vienen de la odometria

		self.delta_x = 0
		self.delta_y = 0

		# initial coordenates
		self.x = 1.7
		self.y = 0.27
		self.theta = 0

		self.mx = 0
		self.my = 0

		self.flag_properties_recevied = ""

		self.ID = 0

		# initialize the odometry get the error of robot's pose
		self.odom = Odometry()

		# Init Publishers

		self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)


		# Initialize psubscribers
		rospy.Subscriber("wl", Float32, self.wl_cb)
		rospy.Subscriber("wr", Float32, self.wr_cb)
		rospy.Subscriber('distanceFromRobot', Float64, self.getDistanceFromRobot)
		rospy.Subscriber('angleFromRobot', Float64, self.getAngleFromRobot)
		rospy.Subscriber('whoIDIam', Int64, self.getID)
		rospy.Subscriber('flagNotArucoDetected', String, self.getFlag)

		# Robot constants  
		self.r = 0.05  # puzzlebot wheel radius [m] 
		self.L = 0.19  # puzzlebot wheel separation [m] 
		self.dt = 0.1  # Desired time to update the robot's pose [s] 

		# variables to correction step
		self.zrik = 0
		self.zraik = 0
	
		# Variables 
		self.w = 0.0  # Robot's angular speed [rad/s] 
		self.v = 0.0  # Robot's linear speed [m/s]  
		self.theta = 0.0  # Robot's orientation [rad]  
		self.wr = 0.0  # Right wheel speed
		self.wl = 0.0  # Left wheel speed



		# Dead reckoning variables
		self.z_covariance = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]) # Sigma
		self.current_z_covariancez_covariance = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]) # Sigma
		self.current_miu = np.array([0,0,0]) # correcion de la posicion del robot en Filtro de Kalman
		self.miu = np.array([0,0,0]) # correcion de la posicion del robot en Filtro de Kalman
		
		# variables para Q
		self.wr_k = 0.15
		self.wl_k = 0.025
		self.w_sigma_const = 0.5 * self.dt * self.r

		
		rate = rospy.Rate(int(1.0/self.dt))  # Set the rate of the while loop

		while not rospy.is_shutdown():

			if self.flag_properties_recevied == "False":
				print("I am in prediction")
				# Get robot velocities
				[self.v, self.w] = self.get_robot_vel(self.wr, self.wl)
			
				#do dead_reackoning
				self.dead_reackoning_prediction()

				#update robot pose
				self.update_robot_pose(self.v, self.w)
				
				# Publish odometry message to visualize in rviz and send to bug0 and bug2
				odom_msg = self.get_odom_stamped(self.x, self.y, self.theta, self.z_covariance, self.v, self.w) 
				self.odom_pub.publish(odom_msg)
			
			if self.flag_properties_recevied == "True":
				print("I am in correction")
				
				self.kalman_correction()
				self.x = self.miu[0]
				self.y = self.miu[1]
				self.theta = self.miu[2]

				# Publish odometry message to visualize in rviz and send to bug0 and bug2
				odom_msg = self.get_odom_stamped(self.x, self.y, self.theta, self.z_covariance, self.v, self.w) 
				self.odom_pub.publish(odom_msg)

			rate.sleep()

	# Update robot pose using the computed velocities
	def update_robot_pose(self, v, w): 
		self.x = self.x + v * np.cos(self.theta) * self.dt
		self.y = self.y + v * np.sin(self.theta) * self.dt
		self.theta = self.theta + w * self.dt
	
	def getDistanceFromRobot(self, zrik):
		self.zrik = zrik.data

	def getAngleFromRobot(self, zraik):
		self.zraik = zraik.data
	
	# Callback for left wheel speed
	def wl_cb(self, msg): 
		self.wl = msg.data

	# Callback for right wheel speed
	def wr_cb(self, msg): 
		self.wr = msg.data

	# Calculate robot velocities from wheel speeds. 
	def get_robot_vel(self, wr, wl): 
		v = ((wl + wr) / 2) * self.r 
		w = ((wr - wl) / self.L) * self.r
		return [v, w] 

	# Get odometry message with stamped pose
	def get_odom_stamped(self, x, y, yaw, sigma, mu_v, mu_w): 
		odom_stamped = Odometry() 
		odom_stamped.header.frame_id = "odom" 
		odom_stamped.child_frame_id = "chassis"
		odom_stamped.header.stamp = rospy.Time.now() 
		odom_stamped.pose.pose.position.x = x
		odom_stamped.pose.pose.position.y = y

		quat = quaternion_from_euler(0, 0, yaw) 
		odom_stamped.pose.pose.orientation.x = quat[0]
		odom_stamped.pose.pose.orientation.y = quat[1]
		odom_stamped.pose.pose.orientation.z = quat[2]
		odom_stamped.pose.pose.orientation.w = quat[3]

		# Construct the covariance matrix
		odom_array = np.array([[sigma[0][0], sigma[0][1], 0, 0, 0, sigma[0][2]],
							   [sigma[1][0], sigma[1][1], 0, 0, 0, sigma[1][2]],
							   [0, 0, 0, 0, 0, 0],
							   [0, 0, 0, 0, 0, 0],
							   [0, 0, 0, 0, 0, 0],
							   [sigma[2][0], sigma[2][1], 0, 0, 0, sigma[2][2]]])
		
		odom_stamped.pose.covariance = odom_array.flatten().tolist()

		odom_stamped.twist.twist.linear.x = mu_v
		odom_stamped.twist.twist.angular.z = mu_w
		
		return odom_stamped 

	# Implement dead reckoning
	def dead_reackoning_prediction(self):
		self.sigma_k = np.array([[self.wr_k * self.wr, 0],
								 [0, self.wl_k * self.wl]])

		self.w_sigma = self.w_sigma_const * np.array([[np.cos(self.miu[2]), np.cos(self.miu[2])],
													   [np.sin(self.miu[2]), np.sin(self.miu[2])],
													   [2 / self.L, -2 / self.L]]) # signo
		self.Q_k =  self.w_sigma.dot(self.sigma_k).dot(self.w_sigma.T)

		self.H = np.array([[1, 0, -(((self.wl + self.wr) * self.w_sigma_const) * np.sin(self.miu[2]))], # verificar inversion de sin y cos
						   [0, 1, (((self.wl + self.wr) * self.w_sigma_const) * np.cos(self.miu[2]))],
						   [0, 0, 1]])

		self.miu = np.array([self.miu[0] + (((self.wl + self.wr) * self.w_sigma_const) * np.cos(self.miu[2])),
							 self.miu[1] + (((self.wl + self.wr) * self.w_sigma_const) * np.sin(self.miu[2])),
							 self.miu[2] + (((self.wr - self.wl) / self.L) * self.w_sigma_const)])

		print("POSICIONES AL NO VER ARUCO (X, Y, THETA): " + str(self.miu))
		
		self.z_covariance = (self.H.dot(self.z_covariance).dot(self.H.T)) + self.Q_k

		print("COVARIANZA AL NO VER ARUCO ~ : " + str(self.z_covariance))
		

	def getID(self, msg): 
		self.ID = msg.data

	def getFlag(self, msg):
		self.flag_properties_recevied = msg.data
	
	def kalman_correction(self):
		
		# Define coordentas for mx and my
		# Mapeo de IDs a coordenadas
		self.id_to_coords = {
			701: (0, 1.60),
			702: (0, 0.80),
			703: (1.73, 0.80),
			704: (2.63, 0.39),
			705: (2.85, 0),
			706: (2.865, 2.0),
			707: (1.735, 1.22)
		}

		if self.ID in self.id_to_coords:
			self.mx, self.my = self.id_to_coords[self.ID]
			print("mx: " + str(self.mx) + "my: " + str(self.my))
			
		
		# FILTRO DE BAYES CON PROPAGACION GAUSSIANA
			
		# definimos nuestro modelo de observacion
		self.delta_x = self.mx - self.miu[0] # we recieve mx of the Aruco and Robot position
		self.delta_y = self.my - self.miu[1] # we recieve my of the Aruco and Robot position
		self.p = (self.delta_x **2) + (self.delta_y **2) # # landmark de la distancia del robot al Aruco (marcador o landmark)

		# DEFINIMOS Y CALCULAMOS NUESTRO MODELO DE OBSERVACION (zk):


		#self.d = np.sqrt((self.x_target - self.x) ** 2 + (self.y_target - self.y) ** 2) # CALCULAR DISTANCIA AL GOAL
		#self.theta_G = np.arctan2(self.y_target - self.y, self.x_target - self.x) # CALCULAR ANGULO THETA G
			
			
		self.zi_k = np.array([np.sqrt(self.p), np.arctan2(self.delta_y, self.delta_x) - self.miu[2]]) # OBSERVATION MODEL (landmark, estimated position)

		# linealizamos nuestro modelo de observacion para propagar las incertidumbres de zik porque tiene ruido el landmark que estoy viendo y la posicion actual donde me encuentro
		# es por ello que se propaga la de la posicion actual para que refleje cambios en la del landmark, que es lo que estoy viendo (Rk).

		self.gk = np.array([[-(self.delta_x/np.sqrt(self.p)), -(self.delta_y/np.sqrt(self.p)), 0],[(self.delta_y/self.p), -(self.delta_x/self.p), -1]])

		self.Rk = np.array([[0.5, 0],[0, 0.3]])
		# como propago la incertidumbre de medicion respecto a la posicion actual de mi robot con la que tiene que llegar al punto final?
		# estoy propagando mi mediciony cuanto varia mi medicion (esta varianza en ocasiones es mas pequena que las demas)
		#esto se puede realizar con el calculo de la varianza de la medicion en zk.
		# nota: Rk es la incertidumbre de la medicion, que es gaussiana. SigmaK gorrito es otra varianza

		self.zk = self.gk.dot(self.z_covariance).dot(self.gk.T) + self.Rk # para sigmak se esta propagando la incertidumbre de la posicion. Gk y Gtk son modelos de medicion


		# ahora dependemos de 2 incertidumbres, de la medicion Sigmacurrent y MiuCurrent (modelo no lineal) y Zi,k (modelo de medicion). Extra: Rk (incertidumbre de medicion)


		# calculamos la ganancia de Kalman para poder escoger entre el modelo no lineal (sigmak, miuk) o en mi medicion (medicion-medicion estimada) (lo que multiplica a kk)
			
		self.kk = self.z_covariance.dot(self.gk.T).dot(np.linalg.inv(self.zk)) # si kk de es muy grande voy a confiar en mi medicion (sensores ext.), si es pequeno en mi modelo no lineal


		# luego nos vamos a encontrar con una nueva incertidumbre (sigmak sin gorrito)
		# donde si kk es muy grande, sigma de k va a ser muy pequena
		# por lo tanto voy a devolver algo mas pequeno que mi incertidumbre de sigma de k (con gorrito) o lo que multiplica a Kk
		# por ende calculamos primero la posicion del robot usando la observacion real, luego la covarianza

		#Antes recibimos la distancia y angulo respectiva

		self.zi_k_measurement = np.array([self.zrik,self.zraik]) # estimate real measurement. distancia y angulo del robot al aruco.

		self.current_miu = self.miu + self.kk.dot(self.zi_k_measurement-self.zi_k) # zi_k_measurement es la observacion real estimada de mi modelo. es current?
		self.current_z_covariance = (self.I-self.kk.dot(self.gk)).dot(self.z_covariance) # I, incertidumbre que tengo.
		
		
		### visualizar que las posiciones al DETECTAR ARUCO - NO DETECTA ARUCO cambia
		
		print("POSICIONES ACTUALIZADASS (X, Y, THETA): " + str(self.current_miu)) # se corrige en todo momento en cuanto vea al Aruco. por lo tanto, corrige la ultima posicion antes de ver al Aruco.
		# su correcion final es cuando deja de ver al Aruco, lo cual debe ser cierto que por donde va luego luego, debe ser la medida en metros en el mapa.

		# visualizar que la covarianza al NO DETECTAR ARUCO INCREMENTE - CUANDO DETECTA VA REDUCIENDO

		print("COVARIANZA MEJORADA Y PEQUENA: " + str(self.current_z_covariance)) # debe ser menor al casi terminar de ver el aruco a cuando apenas iba a ver el aruco 

# Main program
if __name__ == "__main__":  
	Localisation()

