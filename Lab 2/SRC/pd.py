#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32

class TurtlePositionControl:
    def __init__(self):
        rospy.init_node('turtle_x_controller')
        
        # Suscriptor para obtener la posición de la tortuga
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_position)
        
        # Publicador para enviar comandos de movimiento a la tortuga
        self.motion_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Publicadores para monitorear error y velocidad
        self.error_publisher = rospy.Publisher('/turtle/error_x', Float32, queue_size=10)
        self.speed_publisher = rospy.Publisher('/turtle/vel_x', Float32, queue_size=10)
        
        # Frecuencia de actualización (10 Hz)
        self.loop_rate = rospy.Rate(10)
        
        self.current_position = 0

    def update_position(self, pose_data):
        # Callback para actualizar la posición en X de la tortuga
        self.current_position = pose_data.x

    def move_to_target(self, target_x):
        # Ganancia proporcional (ajustable según necesidad)
        proportional_gain = 1

        while not rospy.is_shutdown():
            # Cálculo del error
            position_error = target_x - self.current_position
            
            # Cálculo de la velocidad proporcional
            movement_speed = proportional_gain * position_error
            
            # Publicación de los valores
            self.error_publisher.publish(position_error)
            self.speed_publisher.publish(movement_speed)
            
            # Creación del mensaje de movimiento
            command_msg = Twist()
            command_msg.linear.x = movement_speed
            
            # Envío del comando de velocidad
            self.motion_pub.publish(command_msg)
            
            # Mostrar datos en consola
            rospy.loginfo("Posición actual: %f, Error: %f, Velocidad: %f", self.current_position, position_error, movement_speed)
            
            # Condición de parada cuando se alcanza la posición deseada
            if abs(position_error) < 0.1:
                rospy.loginfo("Objetivo alcanzado")
                break
            
            # Esperar siguiente iteración
            self.loop_rate.sleep()

    def request_target_x(self):
        print("Ingrese la coordenada x de destino:")
        return float(input("Destino x: "))

    def interactive_control(self):
        while not rospy.is_shutdown():
            # Pedir una nueva posición objetivo al usuario
            target_x = self.request_target_x()
            
            # Mover la tortuga al punto indicado
            self.move_to_target(target_x)

if __name__ == '__main__':
    try:
        turtle_control = TurtlePositionControl()
        turtle_control.interactive_control()
    except rospy.ROSInterruptException:
        pass

