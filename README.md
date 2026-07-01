# Arduino_Python_navigator_robot
Solving maze and come back: threads, encoder/odometry, ultrasonic/IR, motions, communication with Pi, scanning logic, path saving, and optimized loop-closure
## Features
- **Multi-Threading:** Real-time task delegation using Mbed RTOS.
- **Sensor Fusion:** Noise-free distance data via filtered Ultrasonic and I2C IR sensors.
- **Odometry & Navigation:** Live dead-reckoning tracking and precise orthogonal alignments.
- **Motion Control:** Smooth, directional locomotion (forward, backward, turning, stopping, and turn 90 degree left or right) mapped to specific speed variables and precise angular maneuvers.
- **Pi Communication:** Asynchronous UART interfacing for high-level commands and telemetry.
- **Scanning Logic:** Obstacle-triggered routines and decision-making fallbacks. (scan every 90 degree, identify the Aruco tags, compare them and go for higher number)
- **Path Optimization:** Circular buffer waypoint logging and a loop-closure algorithm for a pruned, shortened return journey.
### Raspberry Pi (High-Level Python Intel)
* **Computer Vision Core:** Leverages OpenCV to capture and process real-time frames from a Pi Camera, utilizing custom dictionary matching for precise **ArUco marker detection**.
* **Visual Guidance & Mapping:** Dynamically translates detected ArUco marker IDs into high-level directional commands (e.g., navigating grid junctions or identifying the goal) to feed instructions back to the robot.
* **Asynchronous Telemetry Parse:** Runs a multi-threaded Python loop to continuously read incoming UART telemetry from the Arduino, parsing raw metrics into mapped telemetry logs ($X, Y, \theta$, and sensor states).
* **Live Matplotlib Dashboard:** Real-time visualization that graphs the robot's dynamic position coordinates and live sensor data streams, providing immediate telemetry feedback.
