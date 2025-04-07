#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
import math

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

            # DTG y ATG siempre serán 0.0 porque la tortuga está en la posición exacta
            dtg = 0.0
            atg_deg = 0.0

            print(f"\nDistance to Goal (DTG): {dtg:.4f}")
            print(f"Angle to Goal (ATG): {atg_deg:.4f}°")

if __name__ == '__main__':
    main()

