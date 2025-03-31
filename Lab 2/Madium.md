
# Report: Turtlesim Keyboard Control and Shape Drawing

This report documents two functionalities implemented with **turtlesim**:
1. **Keyboard Control:** A node that allows moving the turtle using keyboard inputs (professor’s code).
2. **Shape Drawing:** Examples of nodes that automatically make the turtle draw a square and an equilateral triangle without an interactive controller.

---

## 1. Keyboard Control for Turtlesim

This node enables controlling the turtle by reading key presses directly from the keyboard. The available actions are:
- **x:** Move along the *X* axis.
- **y:** Move along the *Y* axis.
- **s:** Stop the movement.
- **q:** Quit the program.

### Code

```python
import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    """Reads a key from the keyboard without needing to press Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Read a single character
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Control the turtle with the following keys:")
    print("  x -> Move in X direction")
    print("  y -> Move in Y direction")
    print("  s -> Stop moving")
    print("Press 'q' to exit.")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()
        
        if key == 'x':
            msg.linear.x = 2.0  # Move in X direction
        elif key == 'y':
            msg.linear.y = 2.0  # Move in Y direction
        elif key == 's':
            msg.linear.x = 0.0
            msg.linear.y = 0.0  # Stop the movement
        elif key == 'q':  
            print("Exiting...")
            break  # Exit the loop
        
        pub.publish(msg)

if __name__ == '__main__':
    main()
```

### Explanation

- **Initialization:** The node is initialized with `rospy.init_node`, and a publisher is set up on the `/turtle1/cmd_vel` topic to send movement commands.
- **Key Reading:** The `get_key()` function configures the terminal to read a key without requiring the Enter key.
- **Main Loop:** The pressed key is evaluated to configure a `Twist` message, which is then published to control the turtle's movement.
- **Exiting:** Pressing 'q' will break the loop and end the program.

---

## 2. Automatic Shape Drawing with Turtlesim

The following examples demonstrate how to automatically control the turtle to draw specific geometric shapes.

### 2.1 Drawing a Square

This code makes the turtle draw a square by moving forward for each side and then turning 90 degrees.

```python
import rospy
from geometry_msgs.msg import Twist
import time

def draw_square():
    rospy.init_node('turtle_square', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    # Parameters
    side_length = 2.0         # Length of each side
    linear_speed = 2.0        # Linear speed (m/s)
    angular_speed = 1.5708    # Angular speed for a 90° turn (rad/s)
    move_time = side_length / linear_speed  # Time to move one side

    for i in range(4):
        # Move forward
        twist.linear.x = linear_speed
        twist.angular.z = 0.0
        pub.publish(twist)
        time.sleep(move_time)

        # Brief stop
        twist.linear.x = 0.0
        pub.publish(twist)
        time.sleep(0.5)

        # Turn 90°
        twist.angular.z = angular_speed
        pub.publish(twist)
        time.sleep(1)  # Adjust this time to achieve a 90° turn
        twist.angular.z = 0.0
        pub.publish(twist)
        time.sleep(0.5)

    # Stop the turtle
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        draw_square()
    except rospy.ROSInterruptException:
        pass
```

#### Explanation

- **Moving Forward:** The turtle moves forward a distance defined by the side length.
- **Rotation:** An angular speed is applied to rotate the turtle 90° after each side.
- **Loop:** The process repeats 4 times to complete the square.

---

### 2.2 Drawing an Equilateral Triangle

This code makes the turtle draw an equilateral triangle by moving forward and then turning 120° at each vertex.

```python
import rospy
from geometry_msgs.msg import Twist
import time

def draw_triangle():
    rospy.init_node('turtle_triangle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    # Parameters
    side_length = 2.0         # Length of each side
    linear_speed = 2.0        # Linear speed (m/s)
    angular_speed = 2.0944    # Angular speed for a 120° turn (rad/s)
    move_time = side_length / linear_speed  # Time to move one side

    for i in range(3):
        # Move forward
        twist.linear.x = linear_speed
        twist.angular.z = 0.0
        pub.publish(twist)
        time.sleep(move_time)

        # Brief stop
        twist.linear.x = 0.0
        pub.publish(twist)
        time.sleep(0.5)

        # Turn 120°
        twist.angular.z = angular_speed
        pub.publish(twist)
        time.sleep(1)  # Adjust this time to achieve a 120° turn
        twist.angular.z = 0.0
        pub.publish(twist)
        time.sleep(0.5)

    # Stop the turtle
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        draw_triangle()
    except rospy.ROSInterruptException:
        pass
```

#### Explanation

- **Moving Forward:** The turtle advances a distance equal to the side length.
- **Rotation:** A 120° rotation is applied at each vertex.
- **Loop:** This process repeats 3 times to complete the equilateral triangle.

---

## 3. Launch File

To facilitate running the nodes and the **turtlesim** simulator, you can use a ROS launch file. Below is an example of a `launch` file that starts both the **turtlesim** node and the keyboard control node:

```xml
<launch>
    <!-- Launch the turtlesim simulator -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim"/>

    <!-- Launch the keyboard control node -->
    <node pkg="your_package" type="turtle_keyboard_control.py" name="turtle_keyboard_control" output="screen"/>
</launch>
```

#### Explanation

- **Turtlesim Node:** Starts the simulator using the `turtlesim_node`.
- **Keyboard Control Node:** Launches the script for keyboard control. Replace `your_package` with the actual name of your ROS package.

---

## Conclusion

This report documents two key functionalities using **turtlesim**:
- **Keyboard Control:** Enables interactive control of the turtle using keyboard input.
- **Shape Drawing:** Demonstrates how to automate turtle movements to draw a square and an equilateral triangle.

These examples illustrate the control of linear and angular velocities in ROS and provide a foundation for further experimentation with interactive and automated turtle movements.

