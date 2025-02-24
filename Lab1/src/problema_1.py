# Le pido al usuario que ingrese un número entero positivo y lo convierto a entero
n = int(input("Introduce un número entero positivo: "))

# Calculo la suma de los primeros n números usando la fórmula: (n * (n + 1)) / 2
# Uso // para obtener un resultado entero 
suma = (n * (n + 1)) // 2

# Imprimir
print(f"La suma de los primeros {n} enteros positivos es: {suma}")

