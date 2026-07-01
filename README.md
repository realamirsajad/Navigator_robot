# Arduino_navigator_robot
Solving maze and come back: threads, encoder/odometry, ultrasonic/IR, motions, communication with Pi, scanning logic, path saving, and optimized loop-closure
## Features
- **Multi-Threading:** Real-time task delegation using Mbed RTOS.
- **Sensor Fusion:** Noise-free distance data via filtered Ultrasonic and I2C IR sensors.
- **Odometry & Navigation:** Live dead-reckoning tracking and precise orthogonal alignments.
- **Motion Control:** Smooth, directional locomotion (forward, backward, turning, stopping, and turn 90 degree left or right) mapped to specific speed variables and precise angular maneuvers.
- **Pi Communication:** Asynchronous UART interfacing for high-level commands and telemetry.
- **Scanning Logic:** Obstacle-triggered routines and decision-making fallbacks. (scan every 90 degree, identify the Aruco tags, compare them and go for higher number)
- **Path Optimization:** Circular buffer waypoint logging and a loop-closure algorithm for a pruned, shortened return journey.
