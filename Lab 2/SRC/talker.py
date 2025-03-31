import rospy
from std_msgs.msg import String

def talker():
    # Creamos un publicador para el tópico 'chatter'. 
    # Se especifica que el tipo de mensaje es String y que la cola puede almacenar hasta 10 mensajes.
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # Inicializamos el nodo de ROS con el nombre 'talker'.
    # El parámetro anonymous=True permite que ROS asigne un nombre único al nodo, evitando conflictos si hay otros nodos con el mismo nombre.
    rospy.init_node('talker', anonymous=True)
    
    # Definimos la tasa de ejecución del bucle, en este caso 1 Hz (un mensaje por segundo).
    rate = rospy.Rate(1)  # 1hz
    
    # Inicializamos un contador para diferenciar cada mensaje publicado.
    i = 0
    
    # Bucle principal que se ejecuta continuamente hasta que se detenga el nodo (por ejemplo, presionando Ctrl-C).
    while not rospy.is_shutdown():
        # Creamos un mensaje que incluye el contador para identificarlo.
        hello_str = "hello world %s" % i
        
        # Se registra el mensaje en los logs, lo que ayuda a depurar y ver la salida en la consola.
        rospy.loginfo(hello_str)
        
        # Publicamos el mensaje en el tópico 'chatter', al cual otros nodos pueden suscribirse.
        pub.publish(hello_str)
        
        # Se espera el tiempo necesario para mantener la tasa de publicación en 1 Hz.
        rate.sleep()
        
        # Incrementamos el contador para el siguiente mensaje.
        i = i + 1

if __name__ == '__main__':
    try:
        # Se llama a la función talker para iniciar el ciclo de publicación de mensajes.
        talker()
    except rospy.ROSInterruptException:
        # Captura la excepción si se interrumpe el nodo (por ejemplo, con Ctrl-C) y finaliza la ejecución de manera limpia.
        pass
