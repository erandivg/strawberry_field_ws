#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

# Archivos de salida
POSITIONS_FILE = "odom_positions.txt"
VELOCITIES_FILE = "velocidades_reales.txt"

class OdomSaver:
    def __init__(self):
        # Abrir el archivo de posiciones en modo escritura
        self.pos_file = open(POSITIONS_FILE, 'w')
        # Abrir el archivo de velocidades en modo escritura
        self.vel_file = open(VELOCITIES_FILE, 'w')

        # Suscribirse al tópico /odom
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Extraer los valores de posición: x, y, z
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extraer las velocidades: lineal en x y angular en z
        v_linear = msg.twist.twist.linear.x
        v_angular = msg.twist.twist.angular.z

        # Extraer el tiempo del mensaje (timestamp)
        timestamp = msg.header.stamp.to_sec()

        # Guardar la posición en el archivo de posiciones
        self.pos_file.write(f"t={timestamp:.2f} - x={x:.3f}, y={y:.3f}, z={z:.3f}\n")
        rospy.loginfo(f"Posición guardada: t={timestamp:.2f} - x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # Guardar las velocidades en el archivo de velocidades
        self.vel_file.write(f"t={timestamp:.2f} - v_linear={v_linear:.3f}, v_angular={v_angular:.3f}\n")
        rospy.loginfo(f"Velocidades guardadas: t={timestamp:.2f} - v_linear={v_linear:.3f}, v_angular={v_angular:.3f}")

    def shutdown(self):
        # Cerrar ambos archivos al finalizar
        self.pos_file.close()
        self.vel_file.close()
        rospy.loginfo("Archivos TXT cerrados.")

if __name__ == '__main__':
    rospy.init_node('odom_saver', anonymous=True)
    odom_saver = OdomSaver()

    rospy.on_shutdown(odom_saver.shutdown)  # Asegurar el cierre de los archivos al finalizar
    rospy.spin()

