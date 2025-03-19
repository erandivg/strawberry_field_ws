#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time

# Variables globales para almacenar los datos
time_data = {'cmd_vel': [], 'error_topic': [], 'homography_matrix': []}
linear_data = []
angular_data = []
error_data = []
homography_data = {f"h({i},{j})": [] for i in range(3) for j in range(3)}  # Para almacenar cada elemento de la homografía

# Tiempo inicial para calcular tiempo relativo
start_time = time.time()

def cmd_vel_callback(data):
    global time_data, linear_data, angular_data

    current_time = time.time() - start_time
    time_data['cmd_vel'].append(current_time)

    linear_velocity = data.linear.x
    angular_velocity = data.angular.z

    linear_data.append(linear_velocity)
    angular_data.append(angular_velocity)

def error_callback(data):
    global time_data, error_data

    current_time = time.time() - start_time
    time_data['error_topic'].append(current_time)

    error_data.append(data.data)

def homography_callback(data):
    global time_data, homography_data

    current_time = time.time() - start_time
    time_data['homography_matrix'].append(current_time)

    if len(data.data) == 9:  # Asegurar que es una matriz 3x3
        homography_matrix = np.array(data.data, dtype=np.float32).reshape((3, 3))
        for i in range(3):
            for j in range(3):
                homography_data[f"h({i},{j})"].append(homography_matrix[i, j])

def save_data_to_file():
    """Guarda los datos en archivos .txt separados para cada tópico"""
    # Guardar datos de /cmd_vel
    with open("cmd_vel_data.txt", "w") as f:
        f.write("Time(s)\tLinearVelocity(m/s)\tAngularVelocity(rad/s)\n")
        for t, lin, ang in zip(time_data['cmd_vel'], linear_data, angular_data):
            f.write(f"{t:.2f}\t{lin:.4f}\t{ang:.4f}\n")

    # Guardar datos de /error_topic
    with open("error_topic_data.txt", "w") as f:
        f.write("Time(s)\tError\n")
        for t, error in zip(time_data['error_topic'], error_data):
            f.write(f"{t:.2f}\t{error:.4f}\n")

    # Guardar datos de /homography_matrix
    with open("homography_matrix_data.txt", "w") as f:
        f.write("Time(s)\t" + "\t".join(homography_data.keys()) + "\n")
        for idx in range(len(time_data['homography_matrix'])):
            row = [time_data['homography_matrix'][idx]] + [homography_data[key][idx] for key in homography_data.keys()]
            f.write("\t".join(f"{value:.4f}" for value in row) + "\n")

# Funciones de animación
def animate_cmd_vel(i):
    ax1.clear()
    ax2.clear()

    ax1.plot(time_data['cmd_vel'], linear_data, label='Linear Velocity (x)', color='blue')
    ax2.plot(time_data['cmd_vel'], angular_data, label='Angular Velocity (z)', color='red')

    ax1.set_title("Linear Velocity vs Time")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Linear Velocity (m/s)")
    ax1.legend()

    ax2.set_title("Angular Velocity vs Time")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Angular Velocity (rad/s)")
    ax2.legend()

def animate_error(i):
    ax3.clear()

    if error_data:
        ax3.plot(time_data['error_topic'], error_data, label='Error Value', color='purple')

    ax3.set_title("Error Value vs Time")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Error")
    ax3.legend()

def animate_homography(i):
    ax4.clear()

    if time_data['homography_matrix']:
        for key, values in homography_data.items():
            ax4.plot(time_data['homography_matrix'], values, label=key)

    ax4.set_title("Homography Elements vs Time")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Value")
    ax4.legend(loc="upper right", fontsize="small")

def main():
    rospy.init_node('multi_topic_plotter', anonymous=True)

    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('/error_topic', Float32, error_callback)
    #rospy.Subscriber('/homography_matrix', Float32MultiArray, homography_callback)
    rospy.Subscriber('/homograpy_numerical', Float32MultiArray, homography_callback)

    global fig1, fig2, fig3, ax1, ax2, ax3, ax4

    # Figuras separadas para cada tópico
    fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
    fig2, ax3 = plt.subplots(figsize=(8, 5))
    fig3, ax4 = plt.subplots(figsize=(10, 6))

    ani1 = FuncAnimation(fig1, animate_cmd_vel, interval=100)
    ani2 = FuncAnimation(fig2, animate_error, interval=100)
    ani3 = FuncAnimation(fig3, animate_homography, interval=100)

    try:
        plt.show()
    finally:
        # Guardar los datos en archivos al cerrar las gráficas
        save_data_to_file()
        rospy.loginfo("Datos guardados en archivos .txt")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
