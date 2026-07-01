# Arduino_navigator_robot
Solving maze and come back: threads, encoder/odometry, ultrasonic/IR, motions, communication with Pi, scanning logic, path saving, and optimized loop-closure
## Features
- **Multi-Threading:** Real-time task delegation using Mbed RTOS.
- **Sensor Fusion:** Noise-free distance data via filtered Ultrasonic and I2C IR sensors.
- **Odometry & Navigation:** Live dead-reckoning tracking and precise orthogonal alignments.
- **Pi Communication:** Asynchronous UART interfacing for high-level commands and telemetry.
- **Scanning Logic:** Obstacle-triggered routines and decision-making fallbacks.
- **Path Optimization:** Circular buffer waypoint logging and a loop-closure algorithm for a pruned, shortened return journey.
