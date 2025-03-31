#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32

class TurtlePIDController:
    def __init__(self):
        # Inicialización del nodo ROS
        rospy.init_node('turtle_pid_controller')
        
        # Suscripción a la posición de la tortuga
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_position)
        
        # Publicadores para enviar comandos y registrar datos
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.error_pub = rospy.Publisher('/turtle/error_x', Float32, queue_size=10)
        self.vel_log_pub = rospy.Publisher('/turtle/vel_x', Float32, queue_size=10)
        
        # Definir la tasa de actualización (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Variables de estado
        self.current_x = 0.0
        self.prev_error = 0.0
        self.error_sum = 0.0
    
    def update_position(self, pose):
        # Callback que actualiza la posición actual de la tortuga
        self.current_x = pose.x

    def move_to_target(self, target_x):
        # Parámetros PID ajustables
        kp = 1.0
        ki = 0.02
        kd = 0.1
        
        while not rospy.is_shutdown():
            # Cálculo de errores para control PID
            error_x = target_x - self.current_x
            self.error_sum += error_x  # Integral del error
            delta_error = error_x - self.prev_error  # Diferencia para el término derivativo
            
            # Cálculo de la señal de control
            control_signal = (kp * error_x) + (ki * self.error_sum) + (kd * delta_error)
            
            # Guardar el error actual para la siguiente iteración
            self.prev_error = error_x
            
            # Creación del mensaje de movimiento
            velocity_cmd = Twist()
            velocity_cmd.linear.x = control_signal
            
            # Publicar los comandos y datos de monitoreo
            self.vel_pub.publish(velocity_cmd)
            self.error_pub.publish(error_x)
            self.vel_log_pub.publish(control_signal)
            
            # Mostrar en la terminal la información del control
            rospy.loginfo(f"Pos: {self.current_x:.2f}, Err: {error_x:.2f}, Vel: {control_signal:.2f}")
            
            # Condición de parada si se alcanza la meta
            if abs(error_x) < 0.05:
                rospy.loginfo("Objetivo alcanzado")
                break
            
            # Pausa antes de la siguiente iteración
            self.rate.sleep()

    def request_target(self):
        # Solicitar al usuario la posición deseada
        return float(input("Ingrese la posición X deseada: "))

    def start_control_loop(self):
        while not rospy.is_shutdown():
            goal_x = self.request_target()
            self.move_to_target(goal_x)

if __name__ == '__main__':
    try:
        controller = TurtlePIDController()
        controller.start_control_loop()
    except rospy.ROSInterruptException:
        pass
