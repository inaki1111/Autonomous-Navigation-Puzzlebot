#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np
#from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Float32, Float64
from tf.transformations import euler_from_quaternion


# This class will make to autocorrect the Puzzlebot every time when a Aruco is detected

class KalmanFilterExtended():
	
	def __init__(self):
		
		# ROBOT CONSTANTS

		self.r = 0.05 # radius [m]
		self.L = 0.19 # puzzlebot wheel separation [m]

		####### 

		self.zrik = 0
		self.zraik = 0 
		self.mx = 0
		self.my = 0

		# posiciones de los marcadores

		# VARIABLES # Our 7 parameters for KFE

		self.mk_minus_k = 0.0 # last position
		self.sigmak_minus_k = 0.0 # 
		self.uk = 0.0 # input data (linear and angular velocities) v and w
		

		# CONDICIONES INICIALES DEL SISTEMA
		
		# valores iniciales de la posicion de nuestro robot
		self.sx = 0
		self.sy = 0
		self.theta = 0

		# posicion inicial de nuestro robot (miu gorrito)
		self.current_miu = np.array[[[float(self.sx)], [float(self.sy)], [float(self.theta)]]]  # note: consider the kalman filter will consider to adapt the most useful little covariance

		self.current_covariance_matrix = np.array([[0,0,0], [0,0,0], [0,0,0]]) # initial covariance matrix

		self.h = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])


		self.Qk = np.array[[[0.5, 0.01, 0.01], [0.01, 0.5, 0.01], [0.01, 0.01, 0.2]]] # motion model covariance matrix ? movimiento del robot
		self.Rk = [[[0.1, 0], [0, 0.02]]] # measurment unceratainty or observation model covariance matrix ? errores de sensor de la camara, de medicion. prueba y error.


		self.delta_t = 0.1 # sampling time
		self.velocity = 1 # [m/s] velocidad linear de nuestro robot movil diferencial
		self.ang_velocity = 0 # [rad/s] mobile robot angular velocity
				

		# VARIABLES A PARTIR DEL USO DE FILTRO DE BAYES CON PROPAGACION GAUSSIANA

		# variables para nuestro zp,k y ztheta,k
		self.delta_x = 0
		self.delta_y = 0
		

		self.zk = np.array[[0,0]]

		self.gk = np.array[[0,0],[0,0]]
		self.theta_G = 0

		self.I = np.array([[1,0,0],[0,1,0],[0,0,1]]) # incertidumbre que se tiene



		# variables que vienen de la odometria

		self.delta_x = 0
		self.delta_y = 0
		self.x_target = 0
		self.y_target = 0

		
		self.x_r = 0 # posicion actual en x de nuestro robot
		self.y_r = 0 # posicion actual en y de nuestro robot

		

		# Other variables
		self.dt = 0.02 # Desired time to update the robot's pose [s]
		
		self.wr = 0 #right wheel speed [rad/s]

		self.wl = 0 #left wheel speed [rad/s]

		self.flag_properties_recevied = False


		# initialize the odometry get the error of robot's pose
		self.odom = Odometry()

		# Init Publishers

		self.odom_pub = rospy.Publisher('odomWithCorrection', Odometry, queue_size=1)

		# Create subscribers

		
		rospy.Subscriber('wl', Float32, self.wl_cb)
		rospy.Subscriber('wr', Float32, self.wr_cb)
		rospy.Subscriber('mx', Float64, self.mx_cb)
		rospy.Subscriber('my', Float64, self.my_cb)
		rospy.Subscriber('odom', Odometry, self.getRobotPositionWithError)
		rospy.Subscriber('distanceFromRobot', Float64, self.getDistanceFromRobot)
		rospy.Subscriber('angleFromRobot', Float64, self.getAngleFromRobot)

		while rospy.get_time() == 0:
			print("no simulated time has been received yet")
		print("Got time")
		

		rate = rospy.Rate(int(1.0/self.dt)) # The rate of the while will be the inverse of the desired delta_t

		while not rospy.is_shutdown():
			
			# Get robot velocities

			self.getRobotVelocities()

			# Do Kalman Filter

			self.current_miu, self.current_covariance_matrix = self.efk_localisation()

			self.x_r = self.current_miu[0] # actual x position at robot
			self.y_r = self.current_miu[1] # actual y position at robot
			self.theta = self.current_miu[2] # actual theta position at robot


			# Sent odometry to bug_0 to correct its position while the Puzzlebot moves

			self.odom_pub.publish(self.x_r, self.y_r, self.theta)
		


			rate.sleep()

	def efk_localisation(self): # THIS FUNCTION RECEIVES LOCALISATION PARAMETERS
		
		
		# ESTIMATE THE ROBOT POSITION (Miu k1)
		
		self.current_miu[0]  += self.delta_t * self.v * np.cos((self.current_miu[2]))
		self.current_miu[1] += self.delta_t * self.v * np.sin((self.current_miu[2]))
		self.current_miu[2] += self.delta_t * self.w


		# PROPAGATE THE ESTIMATED POSITION TO REDUCE THE NOISE (H1). Esta incertidumbre se propaga en las posiciones del robot tambien.
		# Hk is our linealized model every step

		self.h[0][2] = -self.delta_t * self.v * np.sin((self.miu[2]))
		self.h[1][2] = self.delta_t * self.v * np.cos((self.miu[2]))

		# CALCULATE THE UNCERTAINTY PROPAGATION USING THE LINEAL MODEL (COVARIANCE MATRIX (Sigma k+1)).
		# THIS ALLOWS US TO PROPAGATE THE UNCERTAINITY OF ROBOT POSITION (hk)
		# Qk adds more noise to our system

		self.current_covariance_matrix = self.h.dot(self.current_covariance_matrix).dot(self.h.T) + self.Qk
		# HASTA ESTE PASO SE CALCULA LA ELIPSE DE CONFIDENCIA
		
		#---------------------------#
		
		# CONDICION PARA ENCONTRAR EL ARUCO...
		
		if self.flag_properties_recevied == True:

			# FILTRO DE BAYES CON PROPAGACION GAUSSIANA
			
			# definimos nuestro modelo de observacion
			self.delta_x = self.mx - self.x_r # we recieve mx of the Aruco and Robot position
			self.delta_y = self.my - self.y_r # we recieve my of the Aruco and Robot position
			self.p = (self.delta_x **2) + (self.delta_y **2) # 

			# DEFINIMOS Y CALCULAMOS NUESTRO MODELO DE OBSERVACION (zk):

			# landmark de la distancia del robot al Aruco (marcador o landmark)
			self.d = np.sqrt((self.x_target - self.x_r) ** 2 + (self.y_target - self.y_r) ** 2) # CALCULAR DISTANCIA AL GOAL
			self.theta_G = np.arctan2(self.y_target - self.y_r, self.x_target - self.x_r) # CALCULAR ANGULO THETA G
			
			
			self.zi_k = np.array([[self.d],[np.arctan2(self.delta_y, self.delta_x) - self.theta_G]]) # OBSERVATION MODEL (landmark, estimated position)

			# linealizamos nuestro modelo de observacion para propagar las incertidumbres de zik porque tiene ruido el landmark que estoy viendo y la posicion actual donde me encuentro
			# es por ello que se propaga la de la posicion actual para que refleje cambios en la del landmark, que es lo que estoy viendo (Rk).

			self.gk = np.array([[-(self.delta_x/np.sqrt(self.p)), -(self.delta_y/np.sqrt(self.p)), 0],
							[(self.delta_y/self.p), -(self.delta_x/self.p), -1]])

			# como propago la incertidumbre de medicion respecto a la posicion actual de mi robot con la que tiene que llegar al punto final?
			# estoy propagando mi mediciony cuanto varia mi medicion (esta varianza en ocasiones es mas pequena que las demas)
			#esto se puede realizar con el calculo de la varianza de la medicion en zk.
			# nota: Rk es la incertidumbre de la medicion, que es gaussiana. SigmaK gorrito es otra varianza

			self.zk = self.gk.dot(self.current_covariance_matrix).dot(self.gk.T) + self.Rk # para sigmak se esta propagando la incertidumbre de la posicion. Gk y Gtk son modelos de medicion


			# ahora dependemos de 2 incertidumbres, de la medicion Sigmacurrent y MiuCurrent (modelo no lineal) y Zi,k (modelo de medicion). Extra: Rk (incertidumbre de medicion)


			# calculamos la ganancia de Kalman para poder escoger entre el modelo no lineal (sigmak, miuk) o en mi medicion (medicion-medicion estimada) (lo que multiplica a kk)
			
			self.kk = self.current_covariance_matrix.dot(self.gk.dot.T).dot(self.zk.T) # si kk de es muy grande voy a confiar en mi medicion (sensores ext.), si es pequeno en mi modelo no lineal


			# luego nos vamos a encontrar con una nueva incertidumbre (sigmak sin gorrito)
			# donde si kk es muy grande, sigma de k va a ser muy pequena
			# por lo tanto voy a devolver algo mas pequeno que mi incertidumbre de sigma de k (con gorrito) o lo que multiplica a Kk
			# por ende calculamos primero la posicion del robot usando la observacion real, luego la covarianza

			#Antes recibimos la distancia y angulo respectiva

			self.zi_k_measurement = np.array[[[self.zrik,self.zraik]]] # estimate real measurement. distancia y angulo del robot al aruco.

			self.current_miu = self.current_miu + self.kk((self.zi_k_measurement-self.zi_k)) # zi_k_measurement es la observacion real estimada de mi modelo. 
			self.current_covariance_matrix = ((self.I-self.kk).dot(self.gk)).dot(self.covariance_matrix) # I, incertidumbre que tengo.

			return self.current_miu, self.current_covariance_matrix

	def wl_cb(self, wl): # FUNCTION WHO RECIEVES WL VELOCITY FROM ROBOT
		self.wl = wl.data

	def wr_cb(self, wr): # FUNCTION WHO RECIEVES WR VELOCITY FROM ROBOT
		self.wr = wr.data

	def getDistanceFromRobot(self, zrik):
		self.zrik = zrik.data

	
	def getAngleFromRobot(self, zraik):
		self.zraik = zraik.data

	
	def getAngleFromRobot(self, zraik): #Callback del flag
		self.zraik = zraik.data


	def getRobotVelocities(self): # FUNCTION WHO TRANSFORM WL AND WR DATA TO CALCULATE LINEAR AND ANGULAR VELOCITIES
		self.v = self.r * (self.wr + self.wl) / 2.0
		self.w = self.r * (self.wr - self.wl) / self.L

	def getRobotPositionWithError(self, odom): # FUNCTION WHO WILL RECIEVE ROBOT POSITIONS WITH ERROR
		self.x_r = odom.pose.pose.position.x # Position x of the robot with error
		self.y_r = odom.pose.pose.position.y # '''

		# OBTENER QUATERNION
		orientation_q = odom.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.z, orientation_q.w]

		#CONVERTIR QUATERNION
		_, _, self.theta_r = euler_from_quaternion(orientation_list)  # '''

if __name__ == "__main__":
	rospy.init_node("kalman_filter_extended", anonymous=True)
	KalmanFilterExtended()