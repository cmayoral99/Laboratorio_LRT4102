#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import turtle

# Función para mover la tortuga y dibujar el número 1
def dibujar_numero_1():
    # Inicialización de la ventana y la tortuga
    screen = turtle.Screen()
    screen.bgcolor("white")  # Fondo blanco
    screen.title("Tortuga escribiendo el 1")
    
    t = turtle.Turtle()
    t.speed(5)  # Velocidad de la tortuga
    
    # Dibujar el número 1
    t.penup()
    t.goto(-50, -50)  # Posicionar la tortuga
    t.pendown()
    
    # Movimientos para dibujar un 1
    t.left(90)  # Gira 90 grados a la izquierda
    t.forward(100)  # Dibuja una línea recta hacia arriba
    t.right(90)  # Gira 90 grados a la derecha
    t.forward(20)  # Dibuja la parte inferior del 1
    t.backward(40)  # Regresa la tortuga hacia el otro lado
    t.forward(20)  # Dibuja la parte inferior del 1

    # Mantener la ventana abierta hasta que se cierre
    screen.mainloop()

# Llamar a la función para dibujar el número
dibujar_numero_1()
