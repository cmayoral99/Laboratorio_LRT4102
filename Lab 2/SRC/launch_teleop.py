<launch>
    
    <!-- Lanzar el nodo turtlesim_node del paquete turtlesim -->
     
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node" output="screen"/>

    <!-- Lanzar el script teleop.py del paquete Practicas_lab -->
     <!-- abre turtlesim / carpeta donde está / nombre y nodo-->
    <node pkg="Practicas_lab" type="teleop.py" name="teleop_node" output="screen"/>
</launch>
