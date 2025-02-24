import random  # Importo el módulo random para generar números aleatorios
from collections import deque  # Importo deque para crear una cola y hacer la búsqueda más eficiente

# Defino el tamaño del laberinto (5x5)
N = 5

# Creo una matriz (lista de listas) de tamaño 5x5 llena de "o", que representa espacios libres
matriz = [["o" for _ in range(N)] for _ in range(N)]

# Genero un número aleatorio de obstáculos (entre 5 y 10)
num_obstaculos = random.randint(5, 10)  # Cantidad aleatoria de obstáculos

# Coloco obstáculos ("X") en posiciones aleatorias del laberinto
for _ in range(num_obstaculos):
    # Genero coordenadas aleatorias para el obstáculo
    x, y = random.randint(0, N-1), random.randint(0, N-1)
    # Evito colocar un obstáculo en la entrada (0,0) o en la salida (N-1, N-1)
    if (x, y) != (0, 0) and (x, y) != (N-1, N-1):
        matriz[x][y] = "X"  # Coloco el obstáculo

# Imprimo el mapa inicial del laberinto
print("Mapa inicial:")
for fila in matriz:
    print(" ".join(fila))  # Uno los elementos de cada fila separados por espacios

# Defino los posibles movimientos:
# (cambio en la fila, cambio en la columna, símbolo que representa la dirección)
movimientos = [(0, 1, "→"),  # Mover a la derecha
               (1, 0, "↓"),  # Mover hacia abajo
               (-1, 0, "↑"), # Mover hacia arriba
               (0, -1, "←")] # Mover hacia la izquierda

# Creo una cola para el algoritmo BFS (Búsqueda en Anchura)
# La cola empieza con la posición inicial (0,0) y una ruta vacía
cola = deque([(0, 0, [])])

# Creo un conjunto para llevar la cuenta de las posiciones visitadas
visitados = set()

# Variable para indicar si se encontró la ruta correcta (inicio a destino)
exito = False

# Variable para almacenar la ruta final si se encuentra
ruta_final = []

# Inicio del algoritmo de búsqueda en anchura (BFS)
while cola:
    # Extraigo el primer elemento de la cola (posición actual y ruta recorrida hasta aquí)
    x, y, ruta = cola.popleft()

    # Si ya visité esta posición, paso a la siguiente iteración
    if (x, y) in visitados:
        continue

    # Marco la posición actual como visitada
    visitados.add((x, y))

    # Compruebo si he llegado al destino (última celda: esquina inferior derecha)
    if (x, y) == (N-1, N-1):
        ruta_final = ruta  # Guardo la ruta que me llevó hasta el destino
        exito = True     # Indico que se encontró una ruta
        break           # Salgo del ciclo ya que terminé la búsqueda

    # Recorro todos los movimientos posibles desde la posición actual
    for dx, dy, flecha in movimientos:
        # Calculo la nueva posición sumando el cambio de fila y columna
        nx, ny = x + dx, y + dy
        # Verifico que la nueva posición esté dentro de los límites del laberinto y sea transitable ("o")
        if 0 <= nx < N and 0 <= ny < N and matriz[nx][ny] == "o":
            # Agrego la nueva posición y la ruta actualizada a la cola
            cola.append((nx, ny, ruta + [(nx, ny, flecha)]))

# Después de buscar, compruebo si se encontró una ruta
if exito:
    # Creo una nueva matriz para mostrar la ruta encontrada, inicialmente llena de espacios
    matriz_ruta = [[" " for _ in range(N)] for _ in range(N)]
    matriz_ruta[0][0] = "S"  # Marco el inicio con "S"
    # Coloco las flechas en las posiciones de la ruta encontrada
    for rx, ry, flecha in ruta_final:
        matriz_ruta[rx][ry] = flecha
    matriz_ruta[N-1][N-1] = "E"  # Marco el destino con "E"

    # Imprimo la ruta encontrada en el laberinto
    print("\nRuta encontrada:")
    for fila in matriz_ruta:
        print(" ".join(fila))
else:
    # Si no se encontró una ruta, muestro un mensaje indicando que es imposible llegar al destino
    print("Imposible llegar al destino.")
