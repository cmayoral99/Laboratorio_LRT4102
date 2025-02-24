#Este código calcula el pago de un trabajador al multiplicar sus horas trabajadas por el costo de 
#cada hora. Permite manejar números decimales para los casos en los que las horas no sean enteras
#o el pago por hora no sea un número entero.

# Solicito al usuario que ingrese el número de horas trabajadas y convierto ese valor a un número decimal (float)
n = float(input("Introduce el número de horas trabajadas: "))

# Solicito al usuario que ingrese el costo que se paga por cada hora trabajada, y lo convierto a float
m = float(input("Introduce el costo por hora: "))

# Calculo el pago total multiplicando el número de horas trabajadas por el costo por hora
pago = n * m

# Muestro en pantalla el pago correspondiente, concatenando la palabra "pesos" al final del mensaje
print(f"El pago correspondiente es: {pago}" " pesos")
