#El código solicita ingresar un número entero en la consola de Python, se guarda en la variable `n` y
#luego calcula la suma de los números del 1 hasta ese número `n` usando la fórmula proporcionadaa en la tarea

# Le pido al usuario que ingrese un número entero positivo y lo convierto a entero
n = int(input("Introduce un número entero positivo: "))

# Calculo la suma de los primeros n números usando la fórmula: (n * (n + 1)) / 2
# Uso // para obtener un resultado entero 
suma = (n * (n + 1)) // 2

# Imprimir
print(f"La suma de los primeros {n} enteros positivos es: {suma}")

