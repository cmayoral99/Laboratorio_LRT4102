# Communication between Nodes in ROS: Talker and Listener

## Introduction
In this practice, we developed a ROS package called **Practicas_lab** that implements communication between nodes using the publisher-subscriber pattern. Two essential nodes were created: the *talker*, which publishes messages, and the *listener*, which subscribes to those messages. This basic implementation is fundamental to understanding the distributed architecture in ROS.

## Objectives
- Create a ROS package with the dependencies `rospy`, `roscpp`, and `std_msgs`.
- Implement a publisher node (*talker.py*) that sends periodic messages on the `chatter` topic.
- Implement a subscriber node (*listener.py*) that receives and displays the published messages.
- Compile and run the package to validate the proper functioning of node communication.

---

## Implementation

### Talker Node
The *talker* node publishes messages of type `String` on the `chatter` topic at a frequency of 1 Hz. Each message includes a counter that increments with each cycle.

```python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)  # Frequency of 1 Hz
    i = 0
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % i
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        i += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

**Observations:**
- The use of `anonymous=True` ensures that each instance of the node has a unique name, preventing conflicts.
- The publication occurs periodically at 1 Hz, making it easy to verify data transmission.

---

### Listener Node
The *listener* node subscribes to the `chatter` topic and, through a callback function, processes each received message by printing it to the console.

```python
import rospy
from std_msgs.msg import String

def chatter_callback(message):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", message.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, chatter_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

**Observations:**
- The `rospy.spin()` function keeps the node active, waiting for messages.
- Each received message is processed by `chatter_callback`, confirming that communication is functioning correctly.

---

## Compilation and Execution Process

1. **Package Creation:**  
   The **Practicas_lab** package was created using the following command:
   ```
   catkin_create_pkg Practicas_lab rospy roscpp std_msgs
   ```

2. **File Placement:**  
   The files `talker.py` and `listener.py` were placed in the appropriate directory of the package.

3. **Compilation:**  
   The package was compiled using the command:
   ```
   catkin_make
   ```

4. **Execution of Nodes:**  
   - To start the *talker* node:
     ```
     rosrun Practicas_lab talker.py
     ```
   - To start the *listener* node:
     ```
     rosrun Practicas_lab listener.py
     ```

---

## Results and Comparison

Upon executing both nodes, the following was observed:

| Node      | Function                                  | Outcome Observed                                                |
|-----------|-------------------------------------------|-----------------------------------------------------------------|
| **Talker**   | Publish messages on the `chatter` topic   | "hello world" messages with an incrementing counter were published  |
| **Listener** | Receive and display messages              | Console output showing each received message                      |

---

## Conclusions

- **Asynchronous Communication:**  
  ROS's architecture allows for asynchronous communication between nodes, facilitating data exchange without direct coupling between processes.

- **Decoupling of Nodes:**  
  The publisher-subscriber pattern allows nodes to operate independently, making it easier to scale and develop distributed applications.

- **Functionality Verification:**  
  The proper display of messages on both the *talker* and *listener* consoles confirms that communication has been successfully implemented and that the **Practicas_lab** package is correctly configured.

---

## References
- Official ROS documentation.
- Personal notes and tutorials on implementing nodes in ROS.
- Experiences in developing distributed systems using ROS.
- - APA Reference for ChatGPT used in formatting this markdown:
 OpenAI. (2023). ChatGPT (Mar 14 version) [Large language model]. OpenAI. https://openai.com/chatgpt
```
```
