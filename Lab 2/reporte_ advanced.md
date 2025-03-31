# Position Control for Turtlesim (P, PI, PID)

## Introduction
In this practice, we implemented and compared different types of controllers (P, PI, and PID) to regulate the position of the turtle in the ROS *Turtlesim* simulator. We evaluated the performance of each controller using graphing tools such as *PlotJuggler*.

## Objectives
- Implement P, PI, and PID controllers in *Turtlesim*.
- Analyze the response of each controller and its ability to reach the desired position.
- Compare the performance of each control strategy using graphs.

---

## Implementation

### Proportional Controller (P)
This controller adjusts the velocity based on the current error:

\[ v_x = K_p \times error_x \]

Where:
- \( v_x \) is the linear velocity along the X-axis.
- \( K_p \) is the proportional constant.
- \( error_x \) is the difference between the desired position and the current position.

**Observations:**
- The response is fast, but oscillations or steady-state errors may occur if \( K_p \) is not properly tuned.

---

### Proportional-Integral Controller (PI)
The PI controller introduces an integral term to correct accumulated errors:

\[ v_x = K_p \times error_x + K_i \times \sum error_x \]

Where:
- \( K_i \) is the integral constant.
- \( \sum error_x \) represents the accumulation of errors over time.

**Observations:**
- It improves the elimination of steady-state error.
- It may cause overshoot if \( K_i \) is set too high.

---

### Proportional-Integral-Derivative Controller (PID)
This controller adds a derivative term to predict the behavior of the error and improve stability:

\[ v_x = K_p \times error_x + K_i \times \sum error_x + K_d \times \frac{d(error_x)}{dt} \]

Where:
- \( K_d \) is the derivative constant.
- \( \frac{d(error_x)}{dt} \) is the derivative of the error.

**Observations:**
- It improves stability and reduces overshooting.
- It may be more sensitive to noise in the error signal.

---

## Results and Comparison
To analyze the performance of the controllers, we graphed the error over time using *PlotJuggler*.

| Controller | Stability  | Steady-State Error | Response Time  |
|------------|------------|--------------------|----------------|
| **P**      | Medium     | May persist        | Fast           |
| **PI**     | Good       | Low                | Medium         |
| **PID**    | Excellent  | Minimal            | Fastest        |

**Conclusions:**
- The P controller responds quickly but may leave a residual error.
- The PI controller improves error elimination but can generate overshoot.
- The PID controller offers the best stability and precision.

For future improvements, automatic tuning of the \( K_p \), \( K_i \), and \( K_d \) parameters or exploring other control methods could be implemented.

---

## References
- Official ROS and Turtlesim documentation.
- Control theory lecture notes.
- *PlotJuggler* tutorials for ROS.
- APA Reference for ChatGPT used in formatting this markdown:
 OpenAI. (2023). ChatGPT (Mar 14 version) [Large language model]. OpenAI. https://openai.com/chatgpt
```
