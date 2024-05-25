#!/usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np

class TfBroadcaster():

    def __init__(self):
        rospy.init_node('Cordinate_Transform')

        self.x = 0.0
        self.y = 0.0
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # Cuaternión de orientación inicializado correctamente

        rospy.Subscriber("odom", Odometry, self.odom_cb)
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)

        self.my_tf_br = tf2_ros.TransformBroadcaster()

        r = rospy.Rate(60)  # Frecuencia de publicación de 50 Hz

        while not rospy.is_shutdown():
            # Crear la transformación
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()  # Asegurar una marca de tiempo única
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            # Verificar que el cuaternión es válido antes de publicarlo
            if self.is_valid_quaternion(self.orientation):
                t.transform.rotation.x = self.orientation[0]
                t.transform.rotation.y = self.orientation[1]
                t.transform.rotation.z = self.orientation[2]
                t.transform.rotation.w = self.orientation[3]
                # Broadcast de la transformación
                self.my_tf_br.sendTransform(t)
            else:
                rospy.logwarn("Quaternion inválido recibido: {}".format(self.orientation))

            r.sleep()

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Obtener la orientación en forma de cuaternión del mensaje de odometría
        self.orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

    def laser_cb(self, msg):
        # Esta función recibe un mensaje de tipo LaserScan y calcula la dirección y rango del objeto más cercano
        self.closest_range = min(msg.ranges)
        idx = msg.ranges.index(self.closest_range)
        self.closest_angle = msg.angle_min + idx * msg.angle_increment
        # Limitar el ángulo a [-pi, pi]
        self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle))
        # print("ang" + str(self.closest_angle))
        self.lidar_received = 1

    def is_valid_quaternion(self, q):
        # Un cuaternión válido no debe ser todo ceros y debe tener una magnitud cercana a 1
        norm = np.linalg.norm(q)
        return not (q[0] == 0.0 and q[1] == 0.0 and q[2] == 0.0 and q[3] == 0.0) and (0.999 < norm < 1.001)

if __name__ == '__main__':
    TfBroadcaster()
