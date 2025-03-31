#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32

class TurtlePDController:
    def __init__(self):
        rospy.init_node('turtle_pd_controller')
        
        # Suscripción para recibir la posición de la tortuga
        self.position_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.position_callback)
        
        # Publicador para enviar comandos de movimiento
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Publicadores para seguimiento del error y velocidad
        self.error_publisher = rospy.Publisher('/turtle/error_x', Float32, queue_size=10)
        self.derivative_publisher = rospy.Publisher('/turtle/d_error_x', Float32, queue_size=10)
        
        # Configuración de la tasa de actualización (10 Hz)
        self.loop_rate = rospy.Rate(10)
        
        self.current_x = 0
        self.previous_error = 0

    def position_callback(self, pose_msg):
        # Callback para actualizar la posición en X de la tortuga
        self.current_x = pose_msg.x

    def move_to_goal(self, target_x):
        # Coeficientes de control proporcional y derivativo
        kp = 1.0
        kd = 0.1
        
        while not rospy.is_shutdown():
            # Cálculo del error actual
            error_x = target_x - self.current_x
            
            # Cálculo del término derivativo
            d_error_x = error_x - self.previous_error
            
            # Cálculo de la velocidad usando el controlador PD
            velocity_x = (kp * error_x) + (kd * d_error_x)
            
            # Publicación de valores de error y derivada
            self.error_publisher.publish(error_x)
            self.derivative_publisher.publish(d_error_x)
            
            # Creación del mensaje de velocidad
            twist_msg = Twist()
            twist_msg.linear.x = velocity_x
            
            # Enviar comando de movimiento
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir datos en la consola
            rospy.loginfo("Posición: %f, Error: %f, Velocidad: %f", self.current_x, error_x, velocity_x)
            
            # Actualizar el error anterior
            self.previous_error = error_x
            
            # Condición de parada cuando se llega a la posición deseada
            if abs(error_x) < 0.1:
                rospy.loginfo("Objetivo alcanzado")
                break
            
            # Esperar siguiente iteración
            self.loop_rate.sleep()

    def get_target_x(self):
        print("Ingrese la coordenada x de destino:")
        return float(input("Destino x: "))

    def start_movement(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            target_x = self.get_target_x()
            
            # Mover la tortuga a la ubicación objetivo
            self.move_to_goal(target_x)

if __name__ == '__main__':
    try:
        turtle_pd_control = TurtlePDController()
        turtle_pd_control.start_movement()
    except rospy.ROSInterruptException:
        pass
