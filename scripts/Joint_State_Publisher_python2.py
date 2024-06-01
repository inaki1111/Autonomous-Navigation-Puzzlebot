#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class JointsToPublish():
    def __init__(self):
        # Inicialización de las variables de las ruedas
        self.wl = 0
        self.wr = 0
        self.r = 0.05  # Radio de las ruedas
        self.L = 0.19  # Distancia entre ejes del robot
        self.dt = 0.02  # Paso de tiempo
        self.theta_l = 0
        self.theta_r = 0

        # Inicialización del mensaje de estado de las articulaciones
        self.contJoints = JointState()
        self.init_joints()

        # Inicialización del nodo ROS
        rospy.init_node("Joint_State_Publisher")

        # Subscriptores para los datos de las ruedas
        rospy.Subscriber("wl", Float32, self.wl_cb)
        rospy.Subscriber("wr", Float32, self.wr_cb)

        # Publicador para el estado de las articulaciones
        self.joint_pub_tf = rospy.Publisher("joint_states", JointState, queue_size=1)

        # Tasa de bucle
        self.loop_rate = rospy.Rate(int(1.0 / self.dt))

        rospy.on_shutdown(self.stop)

        # Bucle principal
        self.run()

    def init_joints(self):
        # Configuración inicial del mensaje de estado de las articulaciones
        self.contJoints.header.frame_id = "base_link"
        self.contJoints.name.extend(["base_to_left_w", "base_to_right_w"])
        self.contJoints.position.extend([0.0, 0.0])
        self.contJoints.velocity.extend([0.0, 0.0])
        self.contJoints.effort.extend([0.0, 0.0])

    def wl_cb(self, msg):
        self.wl = msg.data

    def wr_cb(self, msg):
        self.wr = msg.data

    def calculate_wheel_velocities(self):
        # Calcular las velocidades angulares de las ruedas izquierda y derecha
        self.theta_l = self.theta_l + self.wl * self.dt
        self.theta_r = self.theta_r + self.wr * self.dt

        return self.theta_l, self.theta_r

    def run(self):
        try:
            while not rospy.is_shutdown():
                # Calcular las velocidades angulares de las ruedas
                self.theta_l, self.theta_r = self.calculate_wheel_velocities()

                # Actualizar el estado de las articulaciones
                self.contJoints.header.stamp = rospy.Time.now()

                # Asignar las velocidades angulares a las posiciones de las articulaciones
                self.contJoints.position[0] = self.theta_r
                self.contJoints.position[1] = self.theta_l

                # Publicar el estado de las articulaciones
                self.joint_pub_tf.publish(self.contJoints)

                # Esperar y repetir
                self.loop_rate.sleep()

        except rospy.ROSInterruptException:
            pass

    def stop(self):
        # Detener el nodo
        rospy.loginfo("Stopping the JointsToPublish node.")

if __name__ == "__main__":
    JointsToPublish()
