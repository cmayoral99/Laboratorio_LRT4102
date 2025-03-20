#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import turtle

# Función para escribir el ID dentro del entorno de la tortuga
def escribir_id(id_str, velocidad):
    # Configuración de la ventana TurtleSim
    screen = turtle.Screen()
    screen.bgcolor("white")  # Fondo blanco
    screen.title("Tortuga escribiendo el ID")
    
    # Crear la tortuga y configurar la velocidad (1 es lento, 10 es rápido)
    t = turtle.Turtle()
    t.speed(velocidad)
    
    # Escribir el ID
    t.penup()
    t.goto(-150, 0)  # Posicionar la tortuga en la pantalla
    t.pendown()
    t.write(id_str, font=("Arial", 48, "normal"))
    
    # Mantener la ventana abierta hasta que la cerremos
    screen.mainloop()

# Llamada a la función con el ID "175787" y una velocidad de 5
escribir_id("175787", 5)
