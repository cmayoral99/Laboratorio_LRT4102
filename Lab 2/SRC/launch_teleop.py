<launch>
    <!--
      Este nodo lanza el simulador de turtlesim.
      - pkg="turtlesim": Se indica que el nodo pertenece al paquete "turtlesim".
      - type="turtlesim_node": Es el ejecutable que inicia la ventana gráfica del simulador.
      - name="turtlesim_node": Se asigna un nombre único al nodo para identificarlo dentro del sistema ROS.
      - output="screen": Los mensajes de log del nodo se muestran en la pantalla.
    -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node" output="screen"/>

    <!--
      Este nodo lanza el script teleop.py que permite controlar la tortuga mediante el teclado.
      - pkg="Practicas_lab": Se indica que el script se encuentra en el paquete "Practicas_lab".
      - type="teleop.py": Es el archivo Python que contiene el código de control por teclado.
      - name="teleop_node": Se asigna un nombre único al nodo para identificarlo.
      - output="screen": Los mensajes de log del nodo se muestran en la pantalla.
    -->
    <node pkg="Practicas_lab" type="teleop.py" name="teleop_node" output="screen"/>
</launch>
