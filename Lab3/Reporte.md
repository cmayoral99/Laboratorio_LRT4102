# Proyecto: Movimiento de la Tortuga en ROS

Este programa fue creado para mover una tortuga en un entorno simulado utilizando ROS (Robot Operating System). La tortuga se posiciona en las coordenadas que se le proporcionan y calcula dos valores importantes: **Distancia a la Meta (DTG)** y **Ángulo hacia la Meta (ATG)**. Además, cada vez que el usuario ingresa nuevas coordenadas, la tortuga se "matará" y reaparecerá en la nueva ubicación.

## ¿Qué hace el programa?

1. **Entrada de Coordenadas y Ángulo**: El programa pide al usuario ingresar las coordenadas **x** y **y** de la meta, además del ángulo **theta** en grados, que es lo que va a definir la orientación de la tortuga.

2. **Cálculo de la Distancia a la Meta (DTG)**: El programa calcula la **Distancia a la Meta (DTG)** usando la fórmula de distancia Euclidiana:
   \[
   DTG = \sqrt{(x_{\text{meta}} - x_{\text{actual}})^2 + (y_{\text{meta}} - y_{\text{actual}})^2}
   \]
   Esto da la distancia directa entre la posición actual de la tortuga y la meta.

3. **Cálculo del Ángulo hacia la Meta (ATG)**: El **Ángulo hacia la Meta (ATG)** se calcula con la función `atan2` de la librería de Python, la cual toma las diferencias entre las coordenadas **x** y **y** para calcular el ángulo en radianes. Luego lo convierto a grados:
   \[
   ATG = \text{atan2}(y_{\text{meta}} - y_{\text{actual}}, x_{\text{meta}} - x_{\text{actual}})
   \]
   Este ángulo nos indica en qué dirección debe moverse la tortuga para llegar a la meta.

4. **Matar y Crear la Tortuga**: Cada vez que el usuario ingresa nuevas coordenadas, se "mata" la tortuga anterior (si estaba presente) y se genera una nueva tortuga en las coordenadas y el ángulo dados. Esto es necesario para empezar siempre desde una posición controlada y no desde donde estaba antes.

5. **Cálculo de DTG y ATG siempre cero**: 
   - Aunque calculamos **DTG** y **ATG**, en este caso ambos valores son **0.0** después de que la tortuga es creada en la nueva meta. Esto se debe a que, como la tortuga está posicionada en la meta de manera instantánea, la distancia entre la posición de la tortuga y la meta es nula.
   - Por lo tanto, aunque el programa haga los cálculos, **DTG** y **ATG** siempre son cero después de mover la tortuga a la meta.

## Uso del bucle `while`

El bucle `while` se utiliza para permitir que el programa siga corriendo continuamente. Esto le da la posibilidad al usuario de ingresar nuevas coordenadas y ángulos de manera repetitiva, sin tener que reiniciar el programa. Cada vez que se ingresan nuevas coordenadas, el programa repite el proceso de "matar" la tortuga y crearla en la nueva posición, mostrando siempre los nuevos cálculos de **DTG** y **ATG**.

El ciclo **`while True`** asegura que el programa siga funcionando hasta que el usuario decida detenerlo manualmente. Esto facilita probar varias ubicaciones sin tener que reiniciar el programa cada vez.

## Conclusión

Este programa muestra cómo interactuar con el sistema ROS para mover una tortuga en un entorno simulado, al mismo tiempo que calcula la distancia y el ángulo hacia un objetivo dado. Al usar la función `atan2` para el ángulo y la fórmula de distancia Euclidiana para el cálculo de la distancia, el programa realiza cálculos precisos sobre cómo la tortuga debe moverse. Además, el uso del bucle `while` permite que el programa sea interactivo, recibiendo entradas continuas sin tener que reiniciarlo. Finalmente, **DTG** y **ATG** siempre serán **0.0** una vez que la tortuga es colocada en la meta, ya que no hay diferencia entre la posición actual y la meta en ese punto.

## Dependencias

- ROS (Robot Operating System)
- Paquete `turtlesim` para la simulación de la tortuga

