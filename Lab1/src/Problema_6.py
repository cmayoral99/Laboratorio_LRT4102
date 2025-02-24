# Ejercicio: Gestión de Inventario de una Tienda de Mercancía Kpop
# Programa para manejar el emprendimiento de una fanatica del kpop
# Se crean productos (lightsticks, álbumes o photocards)
# con su nombre, precio y cantidad disponible. Disponibles a venta con def vender,
# Permite ver el stock y calcular el valor total del inventario.

# Defino clase para los productos
class Producto:
    def __init__(self, nombre, precio, cantidad):
        # Al crear un producto, guardamos su nombre, precio y cantidad en stock.
        self.nombre = nombre       # Nombre del producto (ej. "Lightstick")
        self.precio = precio       # Precio del producto (ej. 39.99)
        self.cantidad = cantidad   # Cantidad en stock (ej. 10)

    def vender(self, cantidad_vendida):
        # Función para vender productos. Resta la cantidad vendida del stock.
        if cantidad_vendida > self.cantidad:
            # Si se intenta vender más de lo que hay, se muestra un mensaje de error.
            print(f"No hay suficiente stock de {self.nombre} para vender esa cantidad.")
        else:
            # Si hay suficiente stock, se descuenta la cantidad vendida y se muestra un mensaje.
            self.cantidad -= cantidad_vendida
            print(f"Se han vendido {cantidad_vendida} unidades de {self.nombre}.")

    def mostrar_informacion(self):
        # Función para mostrar los detalles del producto.
        print(f"Producto: {self.nombre}")         # Muestra el nombre del producto.
        print(f"Precio: {self.precio}")             # Muestra el precio.
        print(f"Cantidad en stock: {self.cantidad}") # Muestra la cantidad disponible.
        # Muestra si el producto está disponible o agotado.
        if self.cantidad > 0:
            print("Disponibilidad: Disponible")
        else:
            print("Disponibilidad: Agotado")
    
    def valor_producto(self):
        # Calcula el valor total del producto en inventario (precio x cantidad).
        return self.precio * self.cantidad

# Definimos una clase para manejar el inventario de la tienda.
class Inventario:
    def __init__(self):
        # Iniciamos el inventario como una lista vacía donde se guardarán los productos.
        self.productos = []

    def agregar_producto(self, producto):
        # Agrega un producto a la lista de productos del inventario.
        self.productos.append(producto)
        print(f"Producto {producto.nombre} agregado al inventario.")

    def mostrar_inventario(self):
        # Muestra la información de todos los productos del inventario.
        print("Inventario de productos:")
        for producto in self.productos:
            producto.mostrar_informacion()  # Llama a la función de cada producto para ver sus detalles.
            print("-" * 30)  # Imprime una línea para separar la información de cada producto.

    def valor_total_inventario(self):
        # Calcula la suma del valor de todos los productos (precio x cantidad).
        total = 0  # Empezamos con total 0.
        for producto in self.productos:
            total += producto.valor_producto()  # Sumamos el valor de cada producto.
        return total

# ------------------------------
# Uso del sistema de inventario:
# ------------------------------

# Creamos un inventario vacío.
inventario = Inventario()

# Creamos algunos productos para fanáticas de Kpop:
# Por ejemplo, un lightstick, un álbum y una photocard.
producto1 = Producto("Lightstick", 39.99, 10)  # 10 lightsticks a 39.99 cada uno.
producto2 = Producto("Álbum", 29.99, 5)         # 5 álbumes a 29.99 cada uno.
producto3 = Producto("Photocard", 4.99, 2)      # 2 photocards a 4.99 cada una.

# Agregamos estos productos al inventario.
inventario.agregar_producto(producto1)
inventario.agregar_producto(producto2)
inventario.agregar_producto(producto3)

# Mostramos la información actual de todos los productos en el inventario.
inventario.mostrar_inventario()

# Vendemos algunos productos y actualizamos el stock.
producto1.vender(3)  # Vendemos 3 lightsticks.
producto3.vender(1)  # Vendemos 1 photocard.

# Volvemos a mostrar el inventario para ver la actualización en el stock.
inventario.mostrar_inventario()

# Calculamos y mostramos el valor total del inventario , todo en consola.
valor_total = inventario.valor_total_inventario()
print(f"El valor total del inventario es: {valor_total}")
