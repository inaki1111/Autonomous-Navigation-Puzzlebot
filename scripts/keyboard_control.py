#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

# Clase para el control del robot
class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.speed = 0.1  # Velocidad lineal
        self.turn = 0.1   # Velocidad angular

    def getKey(self):
        if os.name == 'nt':
            return msvcrt.getch()
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        try:
            while not rospy.is_shutdown():
                key = self.getKey()
                if key == 'w':
                    x = self.speed
                    y = 0
                    z = 0
                    th = 0
                elif key == 's':
                    x = -self.speed
                    y = 0
                    z = 0
                    th = 0
                elif key == 'a':
                    x = 0
                    y = 0
                    z = 0
                    th = self.turn
                elif key == 'd':
                    x = 0
                    y = 0
                    z = 0
                    th = -self.turn
                else:
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                twist = Twist()
                twist.linear.x = x; twist.linear.y = y; twist.linear.z = z
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
                self.pub.publish(twist)
        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            self.pub.publish(twist)

            if os.name != 'nt':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    controller = KeyboardControl()
    controller.run()
