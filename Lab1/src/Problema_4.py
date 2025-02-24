# Este programa genera un número secreto entre 1 y 10 y te pide adivinarlo.
# Si tu intento es muy bajo o muy alto, te da una pista.
# Sigue pidiendo adivinanzas hasta que aciertes, y al final te dice cuántos intentos usaste.

import random
# Generar un número aleatorio entre 1 y 10
numero_secreto = random.randint(1, 10)

intentos = 0  # Contador de intentos

# Bucle que se repite hasta que adivinas el número secreto
while True:
    # Le pido al usuario que ingrese su adivinanza
    adivinanza = int(input("Adivina el número secreto (entre 1 y 10): "))
    intentos += 1  # Aumento el número de intentos

    # Si adivinas el número secreto, muestro un mensaje y salgo del bucle
    if adivinanza == numero_secreto:
        print(f"¡Felicidades! Adivinaste el número en {intentos} intentos.")
        break
    # Si el número es menor, le digo que es muy bajo
    elif adivinanza < numero_secreto:
        print("Estas por lo bajo, intenta de nuevo.")
    # Si el número es mayor, le digo que es muy alto
    else:
        print("Estas por encima del número secreto, intenta de nuevo.")
