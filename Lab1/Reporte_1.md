# Introduction to Python and Object-Oriented Programming (OOP)

## 1. Basic Concepts in Python

### Variables and Data Types
A variable is a space in memory used to store information.
- **Integers (`int`):** Whole numbers without decimals (e.g., `5`, `100`).
- **Decimals (`float`):** Numbers with a fractional part (e.g., `3.14`, `29.99`).
- **Strings (`str`):** Sequences of characters, like `"Hello World"` or `"Lightstick"`. I use these to display messages or name products.
- **Booleans (`bool`):** True/False values.
- **Lists (`list`):** Ordered collections of items, for example, `[1, 2, 3]` or `["a", "b", "c"]`. I’ve used lists to represent matrices (like in the maze) or to store sequences of moves.
- **Sets (`set`):** Collections of unique items, useful for storing values without duplicates.
- **Dictionaries (`dict`):** Collections of key-value pairs, for example, `{"name": "Ana", "age": 30}`.

### Input and Output Functions
- **`print()`:** Used to display information on the screen.  
  Example:
  ```python
  print("Hello World")
### Input and Output Functions
- **`input()`**: Allows the user to enter data.  
  Example:
  ```python
  name = input("What is your name? ")

### Mathematical Operators
Python includes operators to perform calculations:

- **`+` (addition):** e.g., `2 + 3` equals `5`.
- **`-` (subtraction):** e.g., `5 - 2` equals `3`.
- **`*` (multiplication):** e.g., `4 * 3` equals `12`.
- **`/` (division):** e.g., `10 / 2` equals `5.0`.
- **`//` (integer division):** e.g., `10 // 3` equals `3`.
- **`%` (modulus):** e.g., `10 % 3` equals `1`.
- **`**` (power):** e.g., `2 ** 3` equals `8`.

### Control Structures
These allow you to control the flow of your program:

#### Conditionals (`if`, `elif`, `else`)
Example:
```python
age = 20
if age >= 18:
    print("You are an adult")
else:
    print("You are not an adult")

#### Loops:
- **`for` loop:** Iterates over a sequence.  
  Example:
  ```python
  for i in range(5):
      print("Number", i)

while loop: Repeats a block of code while a condition is true.
Example:
python
Copiar
i = 0
while i < 5:
    print("Value", i)
    i += 1
###Comments and Indentation
Comments:
Comments start with the # symbol and are used to explain the code.
Example:
python
Copiar
###### This is a comment
print("Hello")
Indentation:
###Indentation (using spaces at the beginning of lines) is crucial in Python for defining blocks of code, such as those for functions, conditionals, and loops.
#2. Introduction to Object-Oriented Programming (OOP) in Python
Object-Oriented Programming (OOP) is a style of programming that organizes code into “objects.” Each object contains data (attributes) and actions (methods) it can perform. In my inventory example, I used OOP to organize the code so that each product and the inventory have their own properties and functions.

Classes:
Think of classes as blueprints or molds to create objects. In my inventory example, I defined a Producto class to create products with a name, price, and quantity in stock, and an Inventario class to manage a list of products.

Objects:
Objects are instances of a class. For example:

python
Copiar
producto1 = Producto("Lightstick", 39.99, 10)
Here, producto1 is an object of the Producto class.

Attributes:
These are the properties of an object, like the name, price, or quantity in stock in the inventory example.

Methods:
Methods are functions defined within a class that describe the behavior of an object. For example, in the inventory example, the vender() method updates the stock, the mostrar_informacion() method shows product details, and the valor_producto() method calculates the total value of that product in stock.

Summary of Concepts Used in My Examples
Variables and Data Types:
I used integers, floats, and strings to store information like hours worked, prices, or product names.

Control Structures:
I used for and while loops to repeat actions, such as iterating over a matrix or prompting the user to guess a number until they got it right.

Object-Oriented Programming (OOP):
In my inventory example, I created classes (Producto and Inventario) to organize the code so that each object has its own data and functions. This makes the code more organized and easier to manage.
