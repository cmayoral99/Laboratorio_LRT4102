# Este programa calcula el pago total de cada operador en función de su sueldo por hora y las horas trabajadas.

# Lista de operadores con su nombre, sueldo por hora y horas trabajadas. tipo de dato: tupla
operadores = [
    ("Juan", 10, 40),
    ("María", 12, 35),
    ("Pedro", 15, 45),
    ("Ana", 11, 38),
    ("Luis", 13, 42),
    ("Sofía", 14, 39)
]

# Recorremos la lista de operadores
for nombre, sueldo, horas in operadores:
    # En cada iteración, la tupla se descompone automáticamente en las variables nombre, sueldo y horas
    pago = sueldo * horas  # Calculamos el pago total multiplicando sueldo por horas trabajadas
    print(f"{nombre} debe recibir un pago de: {pago}")  # Mostramos el nombre del operador y su pago total en consola
