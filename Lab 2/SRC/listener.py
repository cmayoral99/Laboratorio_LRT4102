import rospy
from std_msgs.msg import String

def chatter_callback(message):
    # Esta función se ejecuta cada vez que se recibe un mensaje en el tópico "chatter".
    # Se utiliza para procesar y mostrar el contenido del mensaje.
    
    # get_caller_id(): obtiene el nombre completo y resuelto del nodo local.
    # Se concatena ese identificador con el mensaje recibido y se registra en los logs.
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", message.data)
    
def listener():
    # Función principal del nodo "listener" que se encarga de inicializar el nodo
    # y suscribirse al tópico "chatter".
    
    # Inicializa el nodo de ROS llamado 'listener'. El parámetro anonymous=True permite
    # que ROS genere un nombre único para este nodo, evitando conflictos si se inician
    # múltiples nodos con el mismo nombre.
    rospy.init_node('listener', anonymous=True)

    # Se suscribe al tópico "chatter", especificando que los mensajes son del tipo String.
    # Cada vez que se reciba un mensaje, se llamará a la función chatter_callback.
    rospy.Subscriber("chatter", String, chatter_callback)

    # rospy.spin() mantiene el nodo en ejecución y evita que el script termine,
    # permitiendo que el nodo siga recibiendo mensajes hasta que se detenga manualmente.
    rospy.spin()

if __name__ == '__main__':
    # Punto de entrada del script: se llama a la función listener para inicializar el nodo.
    listener()
