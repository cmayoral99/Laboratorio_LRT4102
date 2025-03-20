#!/usr/bin/env python

import turtle

# Configuración de la ventana
screen = turtle.Screen()
screen.bgcolor("white")  # Fondo blanco
screen.title("Tortuga escribiendo el 1")

# Crear la tortuga
t = turtle.Turtle()
t.speed(5)  # Velocidad de la tortuga

# Escribir el número 1
t.penup()
t.goto(-50, 0)  # Posicionar la tortuga
t.pendown()

# Dibujar el número 1
t.left(90)
t.forward(100)
t.right(90)
t.forward(20)
t.backward(40)
t.forward(20)

# Mantener la ventana abierta hasta que la cerremos
screen.mainloop()
