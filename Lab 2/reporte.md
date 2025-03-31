# Control de Posición para Turtlesim (P, PI, PID)

## Introducción
En esta práctica, implementamos y comparamos distintos tipos de controladores (P, PI y PID) para regular la posición de la tortuga en el simulador *Turtlesim* de ROS. Evaluamos el desempeño de cada controlador mediante herramientas de graficación como *PlotJuggler*.

## Objetivos
- Implementar controladores de tipo P, PI y PID en *Turtlesim*.
- Analizar la respuesta de cada controlador y su capacidad de alcanzar la posición deseada.
- Comparar el desempeño de cada estrategia de control mediante gráficas.

---

## Implementación
### Controlador Proporcional (P)
Este controlador ajusta la velocidad en función del error presente:

\[ v_x = K_p \times error_x \]

Donde:
- \( v_x \) es la velocidad lineal en el eje X.
- \( K_p \) es la constante proporcional.
- \( error_x \) es la diferencia entre la posición deseada y la actual.

**Observaciones:**
- La respuesta es rápida, pero pueden presentarse oscilaciones o errores en estado estable si \( K_p \) no es adecuado.

---

### Controlador Proporcional-Integral (PI)
El controlador PI introduce un término integral para corregir errores acumulados:

\[ v_x = K_p \times error_x + K_i \times \sum error_x \]

Donde:
- \( K_i \) es la constante integral.
- \( \sum error_x \) es la acumulación de errores a lo largo del tiempo.

**Observaciones:**
- Mejora la eliminación del error en estado estable.
- Puede generar sobreimpulso si \( K_i \) es demasiado grande.

---

### Controlador Proporcional-Integral-Derivativo (PID)
Este controlador añade un término derivativo para predecir el comportamiento del error y mejorar la estabilidad:

\[ v_x = K_p \times error_x + K_i \times \sum error_x + K_d \times \frac{d(error_x)}{dt} \]

Donde:
- \( K_d \) es la constante derivativa.
- \( \frac{d(error_x)}{dt} \) es la derivada del error.

**Observaciones:**
- Mejora la estabilidad y reduce la sobreoscilación.
- Puede ser más sensible al ruido en la señal de error.

---

## Resultados y Comparación
Para analizar el desempeño de los controladores, graficamos el error en función del tiempo en *PlotJuggler*.

| Controlador | Estabilidad | Error en estado estable | Tiempo de respuesta |
|-------------|------------|----------------------|-----------------|
| **P**       | Media      | Puede persistir     | Rápido         |
| **PI**      | Buena      | Bajo                | Medio          |
| **PID**     | Excelente  | Mínimo             | Más rápido    |

**Conclusiones:**
- El controlador P responde rápido pero puede dejar un error residual.
- El PI mejora la eliminación del error pero puede generar sobreimpulso.
- El PID ofrece la mejor estabilidad y precisión.

Para futuras mejoras, se podría implementar un ajuste automático de los parámetros \( K_p, K_i, K_d \) o probar otros métodos de control.

---

## Referencias
- Documentación oficial de ROS y Turtlesim.
- Apuntes de teoría de control.
- Tutoriales de *PlotJuggler* para ROS.

---

