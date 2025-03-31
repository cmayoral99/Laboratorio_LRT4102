# Comunicación entre Nodos en ROS: Talker y Listener

## Introducción
En esta práctica se desarrolló un paquete ROS denominado **Practicas_lab** que implementa la comunicación entre nodos a través del patrón publicador-suscriptor. Se crearon dos nodos esenciales: el *talker*, que publica mensajes, y el *listener*, que se suscribe a dichos mensajes. Esta implementación básica es fundamental para comprender la arquitectura distribuida en ROS.

## Objetivos
- Crear un paquete ROS con las dependencias `rospy`, `roscpp` y `std_msgs`.
- Implementar un nodo publicador (*talker.py*) que envía mensajes periódicos en el tópico `chatter`.
- Implementar un nodo suscriptor (*listener.py*) que recibe y muestra los mensajes publicados.
- Compilar y ejecutar el paquete para validar el correcto funcionamiento de la comunicación entre nodos.

---

## Implementación

### Nodo Talker
El nodo *talker* publica mensajes del tipo `String` en el tópico `chatter` a una frecuencia de 1 Hz. Cada mensaje incluye un contador que se incrementa en cada ciclo.

```python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)  # Frecuencia de 1 Hz
    i = 0
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % i
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        i += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

**Observaciones:**
- Se utiliza `anonymous=True` para asegurar que cada instancia del nodo tenga un nombre único, evitando conflictos.
- La publicación se realiza de manera periódica a 1 Hz, facilitando la verificación de la transmisión de datos.

---

### Nodo Listener
El nodo *listener* se suscribe al tópico `chatter` y, mediante una función de callback, procesa cada mensaje recibido imprimiéndolo en la consola.

```python
import rospy
from std_msgs.msg import String

def chatter_callback(message):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", message.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, chatter_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

**Observaciones:**
- La función `rospy.spin()` mantiene el nodo en ejecución a la espera de mensajes.
- Cada mensaje recibido es procesado por `chatter_callback`, lo que permite confirmar que la comunicación se realiza correctamente.

---

## Proceso de Compilación y Ejecución

1. **Creación del paquete:**  
   Se creó el paquete **Practicas_lab** utilizando el siguiente comando:
   ```
   catkin_create_pkg Practicas_lab rospy roscpp std_msgs
   ```

2. **Colocación de archivos:**  
   Se ubicaron los archivos `talker.py` y `listener.py` en el directorio correspondiente del paquete.

3. **Compilación:**  
   Se compiló el paquete con el comando:
   ```
   catkin_make
   ```

4. **Ejecución de los nodos:**  
   - Para iniciar el nodo *talker*:
     ```
     rosrun Practicas_lab talker.py
     ```
   - Para iniciar el nodo *listener*:
     ```
     rosrun Practicas_lab listener.py
     ```

---

## Resultados y Comparación

Al ejecutar ambos nodos se verificó lo siguiente:

| Nodo      | Función                                      | Resultado Obtenido                                           |
|-----------|----------------------------------------------|--------------------------------------------------------------|
| **Talker**   | Publicar mensajes en el tópico `chatter`   | Mensajes "hello world" con contador incremental publicados   |
| **Listener** | Recibir y mostrar mensajes                  | Impresión en consola de cada mensaje recibido                |

---

## Conclusiones

- **Comunicación Asíncrona:**  
  La arquitectura de ROS permite la comunicación asíncrona entre nodos, facilitando el intercambio de datos sin necesidad de acoplar los procesos directamente.

- **Desacoplamiento de Nodos:**  
  El patrón publicador-suscriptor permite que los nodos operen de manera independiente, lo que facilita la escalabilidad y el desarrollo de aplicaciones distribuidas.

- **Verificación del Funcionamiento:**  
  La correcta visualización de los mensajes en las consolas del *talker* y del *listener* confirma que la comunicación se ha implementado con éxito y que el paquete **Practicas_lab** está configurado de forma adecuada.

---

## Referencias
- Documentación oficial de ROS.
- Apuntes y tutoriales sobre la implementación de nodos en ROS.
- Experiencias personales en el desarrollo de sistemas distribuidos utilizando ROS.
```
