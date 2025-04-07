Claudia Mayoral 175787
Sure! Here's the report translated into English:


# Project: Turtle Movement in ROS

This program was created to move a turtle in a simulated environment using ROS (Robot Operating System). The turtle is positioned at the given coordinates and calculates two important values: **Distance to Goal (DTG)** and **Angle to Goal (ATG)**. Additionally, every time the user enters new coordinates, the turtle will be "killed" and will reappear at the new location.

## What does the program do?

1. **Input Coordinates and Angle**: The program asks the user to input the **x** and **y** coordinates of the goal, as well as the **theta** angle in degrees, which defines the orientation of the turtle.

2. **Calculation of Distance to Goal (DTG)**: The program calculates the **Distance to Goal (DTG)** using the Euclidean distance formula:
   ```
   DTG = √((x_goal - x_current)² + (y_goal - y_current)²)
   ```
   This formula calculates the direct distance between the turtle's current position and the goal.

3. **Calculation of Angle to Goal (ATG)**: The **Angle to Goal (ATG)** is calculated using Python's `atan2` function, which takes the differences between the **x** and **y** coordinates to calculate the angle in radians. It is then converted to degrees:
   ```
   ATG = atan2(y_goal - y_current, x_goal - x_current)
   ```
   This angle tells us in which direction the turtle needs to move to reach the goal.

4. **Killing and Creating the Turtle**: Every time the user enters new coordinates, the previous turtle (if present) is "killed" and a new turtle is created at the given coordinates and angle. This is necessary to always start from a controlled position and not from where the turtle was previously.

5. **DTG and ATG Always Zero**: 
   - Although we calculate **DTG** and **ATG**, in this case both values are **0.0** after the turtle is created at the new goal. This is because, as the turtle is positioned at the goal instantly, the distance between the turtle's position and the goal is zero.
   - Therefore, even though the program performs the calculations, **DTG** and **ATG** are always zero after moving the turtle to the goal.

## Use of the `while` Loop

The `while` loop is used to allow the program to run continuously. This gives the user the ability to input new coordinates and angles repeatedly without needing to restart the program. Each time new coordinates are entered, the program repeats the process of "killing" the turtle and creating it at the new position, always showing the new **DTG** and **ATG** calculations.

The **`while True`** loop ensures the program keeps running until the user decides to stop it manually. This makes it easier to test different locations without restarting the program each time.

## Conclusion

This program demonstrates how to interact with the ROS system to move a turtle in a simulated environment while also calculating the distance and angle to a given goal. By using the `atan2` function for the angle and the Euclidean distance formula for calculating distance, the program performs precise calculations on how the turtle should move. Additionally, using the `while` loop allows the program to be interactive, receiving continuous inputs without needing to restart. Finally, **DTG** and **ATG** will always be **0.0** once the turtle is placed at the goal, as there is no difference between the turtle's current position and the goal at that point.

## Dependencies

- ROS (Robot Operating System)
- `turtlesim` package for turtle simulation
