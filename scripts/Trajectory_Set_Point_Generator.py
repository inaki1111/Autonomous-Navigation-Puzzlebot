#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class SquareClass():
    def __init__(self):
        rospy.init_node("Trajectory_Set_Point_Generator", anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.on_shutdown(self.cleanup)
        
        # Definir la velocidad lineal y angular para moverse en forma de cuadrado
        self.linear_speed = 0.2  # Velocidad lineal [m/s]
        self.angular_speed = 0.5  # Velocidad angular [rad/s]

        # Definir la duración de cada movimiento
        self.linear_duration = 5.0  # Duración de movimiento lineal [s]
        self.angular_duration = 3.14  # Duración de giro [s]

        # Inicializar el mensaje Twist
        self.vel_msg = Twist()

        # Ejecutar el movimiento del cuadrado
        self.run_square()

    def run_square(self):
        # Definir el bucle para moverse en forma de cuadrado
        while not rospy.is_shutdown():
            # Mover hacia adelante
            self.vel_msg.linear.x = self.linear_speed
            self.vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(self.vel_msg)
            rospy.sleep(self.linear_duration)

            # Girar
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(self.vel_msg)
            rospy.sleep(self.angular_duration)

    def cleanup(self):
        # Detener el robot antes de finalizar el nodo
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel_msg)
        rospy.loginfo("Stopping the robot before shutdown.")

if __name__ == "__main__":
    try:
        SquareClass()
    except rospy.ROSInterruptException:
        pass
