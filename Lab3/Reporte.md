Claudia Mayoral 175787

## Part 1
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

### Part 2
# Project: Proportional Control of the Turtle in ROS

In this project, the goal is to control the movement of a turtle in a simulated environment using ROS (Robot Operating System). A proportional controller is used to move the turtle towards a goal specified by the user, without the need to "kill" and "spawn" the turtle at each new position. The controller adjusts the turtle's speed based on the distance and angle to the goal.

## What does the program do?

1. **Input coordinates and angle**: The program asks the user to input the **x** and **y** coordinates of the goal, as well as the **theta** angle (in degrees) so that the turtle is properly oriented towards the goal.

2. **Proportional Control**:
   - **Distance to Goal (DTG)**: It is calculated using the Euclidean distance formula between the turtle's current position and the goal.
     ```
     DTG = √((x_goal - x_current)² + (y_goal - y_current)²)
     ```
   - **Angle to Goal (ATG)**: It is calculated using the `atan2` function, which gives the angle in radians towards the goal, and then converts it to degrees:
     ```
     ATG = atan2(y_goal - y_current, x_goal - x_current)
     ```

3. **Movement towards the Goal**: Using a proportional controller, the program adjusts the turtle's speed based on the distance error (DTG) and the angle error (ATG). The error is multiplied by proportional constants (`Kp_linear` for distance and `Kp_angular` for angle), thus adjusting the turtle's velocity.

4. **Rotation towards the desired angle**: After moving the turtle to the goal, the program rotates the turtle to the desired angle using the same proportional control principle, with a constant `Kp_theta`.

5. **Stopping when the goal is reached**: When the turtle is sufficiently close to the goal (when **DTG** is less than 0.1), the program stops the movement and rotation.

## Use of Proportional Control

**Proportional control** is used to adjust the turtle's velocities based on distance and angle errors:
- For distance, the program uses a proportional constant **`Kp_linear`** to control the linear velocity.
- For the angle, **`Kp_angular`** is used to control the angular velocity, making the turtle rotate towards the goal.

This type of control ensures the turtle moves smoothly and efficiently towards the goal, correcting its path at each iteration.

## Program Flow

1. The program starts by asking the user for the coordinates of the goal and the desired angle.
2. The turtle moves towards the goal by calculating **DTG** and **ATG** at each cycle.
3. When the turtle reaches the goal, it stops and then rotates to adjust its orientation according to the desired angle.
4. The process repeats for new goals, allowing the turtle to move and orient itself correctly.

## Conclusion

This project demonstrates how to use proportional control to move and rotate a turtle in a simulated environment using ROS. The controller adjusts the turtle's speed based on distance and angle errors, allowing the turtle to move efficiently towards a user-specified goal. The rotation towards the desired angle is handled using the same proportional approach, ensuring precise orientation. This approach is simple yet effective for navigation and robot control tasks in controlled environments.

## Dependencies

- ROS (Robot Operating System)
- `turtlesim` package for turtle simulation

## Reference

This report was generated with the assistance of **ChatGPT**.
