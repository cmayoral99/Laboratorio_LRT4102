#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
import math

# Función para matar la tortuga
def matar_tortuga(nombre):
    # Esperamos a que el servicio /kill esté disponible
    rospy.wait_for_service('/kill')
    try:
        # Llamamos al servicio para matar la tortuga
        kill = rospy.ServiceProxy('/kill', Kill)
        kill(nombre)
    except rospy.ServiceException:
        rospy.logwarn(f"No se pudo eliminar la tortuga {nombre}, probablemente ya está eliminada.")

# Función para generar la tortuga en una nueva posición
def crear_tortuga(x, y, angulo_deg, nombre):
    rospy.wait_for_service('/spawn')
    try:
        # Convertimos el ángulo de grados a radianes
        angulo_rad = math.radians(angulo_deg)
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, angulo_rad, nombre)
        return x, y, angulo_rad
    except rospy.ServiceException as e:
        rospy.logerr(f"Error al crear la tortuga: {e}")
        return None

# Función principal
def main():
    # Inicializamos el nodo de ROS
    rospy.init_node('mover_tortuga_a_meta', anonymous=True)

    while True:  # El ciclo se repite para pedir nuevas coordenadas continuamente
        # Pedimos al usuario las coordenadas y el ángulo de la meta
        x_meta = float(input("Ingresa la coordenada x de la meta: "))
        y_meta = float(input("Ingresa la coordenada y de la meta: "))
        angulo_meta = float(input("Ingresa el ángulo de la meta (en grados): "))

        # Eliminamos la tortuga si ya existe
        matar_tortuga("turtle1")

        # Creamos la tortuga en la nueva posición
        resultado = crear_tortuga(x_meta, y_meta, angulo_meta, "turtle1")

        if resultado:
            x_actual, y_actual, angulo_actual = resultado

            # Calculamos la Distancia a la Meta (DTG)
            dtg = math.sqrt((x_meta - x_actual)**2 + (y_meta - y_actual)**2)

            # Calculamos el Ángulo hacia la Meta (ATG) en radianes y lo convertimos a grados
            atg_rad = math.atan2((y_meta - y_actual), (x_meta - x_actual))
            atg_deg = math.degrees(atg_rad)

            # Mostramos los resultados
            print(f"\nDistancia a la meta (DTG): {dtg:.4f}")
            print(f"Ángulo hacia la meta (ATG): {atg_deg:.4f}°")

# Ejecutamos la función principal
if __name__ == '__main__':
    main()
