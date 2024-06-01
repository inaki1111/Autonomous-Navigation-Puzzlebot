#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32, Float32, String  # Cambio aquí

import tf2_ros

# ID DE ARUCO A DETECTAR
TARGET_ARUCO_IDS = [701, 702, 703, 704, 705, 706, 707]

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_tf_detector')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.handle_fiducial_transforms)
        self.whoIDIam_pub = rospy.Publisher('whoIDIam', Int32, queue_size=1)  # Cambio aquí
        self.distanceRA_pub = rospy.Publisher('distanceFromRobot', Float32, queue_size=1)  # Cambio aquí
        self.angleRA_pub = rospy.Publisher('angleFromRobot', Float32, queue_size=1)  # Cambio aquí
        self.flagNotArucoDetected_pub = rospy.Publisher('flagNotArucoDetected', String, queue_size=1)

    def handle_fiducial_transforms(self, msg):
        aruco_detected = False  # Cambio aquí
        for fiducial in msg.transforms:
            if fiducial.fiducial_id in TARGET_ARUCO_IDS:
                aruco_detected = True  # Cambio aquí
                t = fiducial.transform.translation
                #print("Fiducial ID: {}".format(fiducial.fiducial_id))
                #print("Position in Camera Frame: (x={}, y={}, z={})".format(t.x, t.y, t.z))

                # Transformación desde la cámara al marco del robot
                transform_matrix = np.array([
                    [0,  0, 1, 0.1],  # 10cm de desplazamiento en el eje x del robot
                    [-1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0,  0, 0, 1]
                ], dtype=np.float32)  # Cambio aquí
                pos_camera = np.array([t.x, t.y, t.z, 1], dtype=np.float32)  # Cambio aquí
                pos_robot = transform_matrix.dot(pos_camera)

                # Publicar la transformación del marco de la cámara al robot
                camera_to_robot_tf = TransformStamped()
                camera_to_robot_tf.header.stamp = rospy.Time.now()
                camera_to_robot_tf.header.frame_id = "base_link"
                camera_to_robot_tf.child_frame_id = "camera"
                camera_to_robot_tf.transform.translation.x = 0.1
                camera_to_robot_tf.transform.translation.y = 0.0
                camera_to_robot_tf.transform.translation.z = 0.0
                camera_to_robot_tf.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(camera_to_robot_tf)

                # Publicar la posición del Aruco respecto al robot
                aruco_transform = TransformStamped()
                aruco_transform.header.stamp = rospy.Time.now()
                aruco_transform.header.frame_id = "base_link"
                aruco_transform.child_frame_id = "aruco_{}".format(fiducial.fiducial_id)
                aruco_transform.transform.translation.x = pos_robot[0]
                aruco_transform.transform.translation.y = pos_robot[1]
                aruco_transform.transform.translation.z = pos_robot[2]
                aruco_transform.transform.rotation.w = 1.0  # Sin rotación
                self.tf_broadcaster.sendTransform(aruco_transform)

                # Cálculo de distancia y ángulo en el plano xy del robot
                distance = np.linalg.norm(pos_robot[:2])
                angle = np.arctan2(pos_robot[1], pos_robot[0]) * 180 / np.pi  # Convertir a grados
                angle_rad = np.arctan2(pos_robot[1], pos_robot[0])  # Ángulo en radianes

                #print("Position in Robot Frame: (x={:.2f}, y={:.2f}, z={:.2f})".format(pos_robot[0], pos_robot[1], pos_robot[2]))
                #print("Distance to Aruco from Robot: {:.2f} meters".format(distance))
                #print("Angle to Aruco from Robot: {:.2f} degrees".format(angle))
                
                self.whoIDIam_pub.publish(int(fiducial.fiducial_id)) # SEND THE NUMBER OF ID DETECTED
                self.distanceRA_pub.publish(distance) # DISTANCE FROM ROBOT FRAME TO ARUCO, in meters
                self.angleRA_pub.publish(angle_rad) # ANGLE FROM ROBOT FRAME TO ARUCO, in radians
            
        #print("llega")
        if aruco_detected:
            self.flagNotArucoDetected_pub.publish("True")
        else:
            self.flagNotArucoDetected_pub.publish("False") # ANGLE FROM ROBOT FRAME TO ARUCO, in radians
            

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = ArucoDetector()
    detector.run()

