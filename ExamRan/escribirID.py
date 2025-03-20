import turtle

def escribir_id(id_str, velocidad):
    # Configurar la pantalla
    pantalla = turtle.Screen()
    pantalla.title("Tortuga escribiendo el ID")
    
    # Crear la tortuga y configurar la velocidad (1 lenta, 10 rápida; 0 es la máxima animación instantánea)
    t = turtle.Turtle()
    t.speed(velocidad)
    
    # Posicionar la tortuga para que el texto se vea centrado
    t.penup()
    t.goto(-150, 0)
    t.pendown()
    
    # Escribir el ID con una fuente y tamaño definidos
    t.write(id_str, font=("Arial", 48, "normal"))
    
    # Ocultar la tortuga y mantener la ventana abierta
    t.hideturtle()
    pantalla.mainloop()

# Llamada a la función con el ID "175787" y velocidad angular (puedes ajustar el valor de velocidad)
escribir_id("175787", 10)

