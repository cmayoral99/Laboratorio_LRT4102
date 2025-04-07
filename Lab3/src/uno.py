#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def kill_turtle(name):
    rospy.wait_for_service('/kill')
    try:
        kill = rospy.ServiceProxy('/kill', Kill)
        kill(name)
    except rospy.ServiceException:
        rospy.logwarn(f"No se pudo eliminar {name}, probablemente ya fue eliminada.")

def spawn_turtle(x, y, theta_deg, name):
    rospy.wait_for_service('/spawn')
    try:
        theta_rad = math.radians(theta_deg)  # Convertir de grados a radianes
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, theta_rad, name)
        return x, y, theta_rad
    except rospy.ServiceException as e:
        rospy.logerr(f"Error al crear la tortuga: {e}")
        return None

def draw_line(x_start, y_start, x_end, y_end):
    # Publicar un mensaje a la tortuga para moverla y dibujar la línea
    turtle_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # Frecuencia de publicación

    # Activar el lápiz
    rospy.wait_for_service('/turtle1/pen')
    pen_service = rospy.ServiceProxy('/turtle1/pen', SetPen)
    pen_service(True, 0, 0, 255, 3)  # Activar lápiz y establecer el color (rojo, grosor 3)

    # Establecer posición inicial de la tortuga
    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0

    # Mover la tortuga a la posición de inicio
    turtle_pub.publish(move_cmd)
    rate.sleep()

    # Mover la tortuga hasta el objetivo
    move_cmd.linear.x = (x_end - x_start) / 10  # Divide el movimiento en pasos pequeños
    move_cmd.angular.z = 0  # No hay rotación, solo movimiento recto

    for _ in range(10):  # Realizar 10 pasos hacia el objetivo
        turtle_pub.publish(move_cmd)
        rate.sleep()

    # Detener el movimiento después de alcanzar la posición final
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    turtle_pub.publish(move_cmd)

def main():
    rospy.init_node('turtle_spawn_goal', anonymous=True)

    while True:  # Esto permitirá que el programa siga ejecutándose indefinidamente
        # Pedir al usuario la posición objetivo (goal)
        x_goal = float(input("Ingresa la coordenada x del goal: "))
        y_goal = float(input("Ingresa la coordenada y del goal: "))
        theta_goal_deg = float(input("Ingresa el ángulo theta del goal (en grados): "))

        # Eliminar la tortuga inicial
        kill_turtle("turtle1")

        # Crear la tortuga en la posición deseada
        result = spawn_turtle(x_goal, y_goal, theta_goal_deg, "turtle1")

        if result:
            x_current, y_current, theta_current = result

            # Calcular Distance to Goal (DTG)
            dtg = math.sqrt((x_goal - x_current)**2 + (y_goal - y_current)**2)

            # Calcular Angle to Goal (ATG) en radianes y luego convertir a grados
            atg_rad = math.atan2((y_goal - y_current), (x_goal - x_current))
            atg_deg = math.degrees(atg_rad)

            print(f"\nDistance to Goal (DTG): {dtg:.4f}")
            print(f"Angle to Goal (ATG): {atg_deg:.4f}°")

            # Dibujar la línea desde la posición anterior hasta la nueva
            draw_line(x_current, y_current, x_goal, y_goal)

if __name__ == '__main__':
    main()
