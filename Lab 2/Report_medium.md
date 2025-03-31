# Practice Report: Keyboard Control in Turtlesim and Drawing Geometric Shapes

## 1. Introduction

In this practice session, a ROS application was developed to interact with the **turtlesim** simulator. The objective was to implement two main functionalities:  
- **Keyboard Control:** Allowing the user to move the turtle in the simulated environment using keyboard commands.  
- **Drawing Shapes:** Automatically drawing a square and an equilateral triangle using programmed movements without interactive input during execution.

## 2. Objectives

- **Implement a Keyboard Control Node:** Using Python and ROS, a node was created that reads keyboard input and sends `Twist` messages to the `/turtle1/cmd_vel` topic to control the turtle's movement.  
- **Draw Geometric Shapes:** Program the sequence of movements required for the turtle to trace a square and an equilateral triangle in the simulator.

## 3. Methodology

### 3.1 Keyboard Control

A Python script named `teleop.py` was developed with the following features:

- **Node Initialization:** The node `turtle_keyboard_control` is initialized using `rospy.init_node`.
- **Message Publishing:** A publisher is created for the `/turtle1/cmd_vel` topic that sends messages of type `Twist` from the `geometry_msgs` package.
- **Keyboard Input Reading:** Using the `termios` and `tty` libraries, the script captures keyboard key presses without requiring the Enter key, allowing immediate response.
- **Command Mapping:**  
  - **Key 'x':** Moves the turtle along the X direction with a linear speed.
  - **Key 'y':** Moves the turtle along the Y direction.
  - **Key 's':** Stops the turtle's movement.
  - **Key 'q':** Exits the program.

### 3.2 Drawing Geometric Shapes

For drawing shapes, command sequences were planned so that the turtle would:

- **Draw a Square:**  
  - Move a fixed distance.
  - Rotate 90°.
  - Repeat the process four times to complete the square.

- **Draw an Equilateral Triangle:**  
  - Move a fixed distance.
  - Rotate 120°.
  - Repeat the process three times to close the triangle.

These movements were programmed to run automatically without user interaction during their execution, demonstrating how to automate drawing tasks in the `turtlesim` environment.

## 4. Implemented Code

### 4.1 Keyboard Control Script (`teleop.py`)

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    """Reads a keyboard key without needing to press Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Reads a single character
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Control the turtle using the following keys:")
    print("  x -> Move along X")
    print("  y -> Move along Y")
    print("  s -> Stop")
    print("Press 'q' to exit.")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()
        
        if key == 'x':
            msg.linear.x = 2.0  # Move along X
        elif key == 'y':
            msg.linear.y = 2.0  # Move along Y
        elif key == 's':
            msg.linear.x = 0.0
            msg.linear.y = 0.0  # Stop movement
        elif key == 'q':  
            print("Exiting...")
            break  # Exit the loop
        
        pub.publish(msg)

if __name__ == '__main__':
    main()
```

This script uses "raw" mode to capture keyboard input directly, allowing for immediate control without requiring the Enter key.

### 4.2 Launch File

The following launch file was used to start both the `turtlesim_node` and the control node:

```xml
<launch>
    <!-- Launch the turtlesim_node from the turtlesim package -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node" output="screen"/>

    <!-- Launch the teleop.py script from the Practicas_lab package -->
    <node pkg="Practicas_lab" type="teleop.py" name="teleop_node" output="screen"/>
</launch>
```

This launch file allows the simultaneous execution of both nodes, integrating the simulation and interactive control.

## 5. Results and Discussion

- **Interactive Control:**  
  The implementation of keyboard control allowed for precise movement of the turtle within the simulator. The direct keyboard input method provided immediate response for controlling speed and direction.

- **Drawing Shapes:**  
  Although the main script presented corresponds to manual control, the necessary logic was integrated for the turtle to draw a square and an equilateral triangle using pre-defined commands. This demonstrates the versatility of `turtlesim` in simulating complex movement sequences based on velocity and rotation commands.

- **ROS Integration:**  
  The correct configuration of the launch file facilitated the startup of the required nodes, showcasing an effective integration of the system components.

## 6. Conclusions

This practice session helped consolidate fundamental ROS concepts such as message publishing on topics and controlling movements in a simulated environment. Key takeaways include:

- **Direct Interaction:** Real-time keyboard input enhances interactive control, which is essential in robotics applications.
- **Task Automation:** Programming movement sequences to draw geometric shapes demonstrates the applicability of ROS in task automation and behavior simulation.
- **Node Integration:** Utilizing a launch file simplifies the simultaneous execution of multiple nodes, providing an integrated experience within the ROS environment.

Overall, this practice was successful and provided a solid foundation for developing interactive and automated applications in robotic environments using ROS and `turtlesim`.
- APA Reference for ChatGPT used in formatting this markdown:
 OpenAI. (2023). ChatGPT (Mar 14 version) [Large language model]. OpenAI. https://openai.com/chatgpt
```
