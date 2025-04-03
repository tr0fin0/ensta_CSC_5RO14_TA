#!/usr/bin/env python3

import rospy
import threading
import sys
import select
from std_msgs.msg import Bool

flag_value = False  # Estado inicial de la bandera

def user_input():
    global flag_value
    while not rospy.is_shutdown():
        # Usamos select para verificar si hay una entrada sin bloquear el nodo
        i, o, e = select.select([sys.stdin], [], [], 0.1)
        if i:
            user_input = sys.stdin.readline().strip().lower()
            if user_input == 't':  # Cambiar el flag con 't'
                flag_value = not flag_value  # Alternar entre True y False
                rospy.loginfo(f"Flag cambiado a: {flag_value}")


def publish_flag():
    global flag_value
    rospy.init_node('flag_publisher', anonymous=True)
    pub = rospy.Publisher('/flag', Bool, queue_size=10)
    rate = rospy.Rate(1)  # Publicar 1 vez por segundo

    # Hilo separado para capturar entrada de usuario sin bloquear ROS
    input_thread = threading.Thread(target=user_input)
    input_thread.daemon = True  # Hace que el hilo termine cuando se cierre el nodo
    input_thread.start()

    while not rospy.is_shutdown():
        flag_msg = Bool()
        flag_msg.data = flag_value
        rospy.loginfo(f"Enviando flag: {flag_msg.data}")
        pub.publish(flag_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_flag()
    except rospy.ROSInterruptException:
        pass
