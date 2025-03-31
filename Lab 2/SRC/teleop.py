#!/usr/bin/env python3

# Importamos la librería rospy para interactuar con ROS.
import rospy
# Importamos Twist, que es el mensaje usado para enviar comandos de velocidad a la tortuga.
from geometry_msgs.msg import Twist
# Importamos módulos para manejo del sistema y configuración del terminal.
import sys
import termios
import tty

def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    # Obtenemos el descriptor del archivo correspondiente a la entrada estándar (stdin).
    fd = sys.stdin.fileno()
    # Guardamos la configuración actual del terminal para poder restaurarla más adelante.
    old_settings = termios.tcgetattr(fd)
    try:
        # Ponemos el terminal en modo "raw" para que la lectura de teclas sea inmediata.
        tty.setraw(fd)
        # Leemos un solo caracter de la entrada del teclado.
        key = sys.stdin.read(1)  # Lee un solo caracter
    finally:
        # Restauramos la configuración original del terminal para evitar problemas posteriores.
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    # Devolvemos el caracter leído.
    return key

def main():
    # Inicializamos el nodo de ROS llamado 'turtle_keyboard_control'. El parámetro anonymous=True
    # permite que se puedan ejecutar múltiples instancias del nodo sin conflictos de nombre.
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    
    # Creamos un publicador que enviará mensajes de tipo Twist al tópico '/turtle1/cmd_vel'
    # que es donde turtlesim espera recibir comandos de movimiento.
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Mostramos las instrucciones de control al usuario en la consola.
    print("Controla la tortuga con las teclas:")
    print("  x -> Mover en X")
    print("  y -> Mover en Y")
    print("  s -> Detenerse")
    print("Presiona 'q' para salir.")

    # Bucle principal que se ejecuta continuamente hasta que ROS se detenga o se presione 'q'.
    while not rospy.is_shutdown():
        # Capturamos la tecla que el usuario presiona.
        key = get_key()
        # Creamos un mensaje Twist vacío que será modificado según la tecla presionada.
        msg = Twist()
        
        # Condicionales para asignar valores al mensaje según la tecla presionada.
        if key == 'x':
            msg.linear.x = 2.0  # Establece velocidad 2.0 en la dirección X (movimiento hacia adelante).
        elif key == 'y':
            msg.linear.y = 2.0  # Establece velocidad 2.0 en la dirección Y (movimiento lateral).
        elif key == 's':
            # Si se presiona 's', se detiene el movimiento poniendo a cero las velocidades.
            msg.linear.x = 0.0
            msg.linear.y = 0.0  # Detiene el movimiento
        elif key == 'q':
            # Si se presiona 'q', se imprime un mensaje de salida y se rompe el bucle.
            print("Saliendo...")
            break  # Sale del loop
        
        # Publicamos el mensaje en el tópico para que la tortuga ejecute el movimiento.
        pub.publish(msg)

# Punto de entrada del script: si se ejecuta este archivo, se llama a la función main().
if __name__ == '__main__':
    main()
