#include <mbed.h>
#include <ARB.h>
#include <Wire.h>
using namespace rtos;

// --- IR Sensor ---
#define IR_SENSOR_ADDRESS      (0x80 >> 1)      // I2C address of IR sensor (shifted for 7-bit addressing)
#define IR_SENSOR_DISTANCE_REG 0x5E             // Register address for distance reading
#define IR_SENSOR_SHIFT_REG    0x35             // Register address for shift value

// --- Motor / Direction ---
typedef enum { CW, CCW } Direction;            // Clockwise and Counter-clockwise directions
typedef enum { A, B } Motor;                   // Motor identifiers (Motor A and Motor B)

// --- Median Filter ---
#define FILTER_SAMPLES 5                       // Number of samples for median filtering

// --- Encoder / Wheel ---
#define ENCODER_PPR 3                          // Pulses Per Revolution of encoder
#define GEAR_RATIO 298                         // Gear ratio of motor
#define WHEEL_DIAMETER_CM 4.8f                 // Diameter of wheel in centimeters
#define PULSES_PER_WHEEL_REV (ENCODER_PPR * GEAR_RATIO)  // Total pulses per wheel revolution
#define PI 3.1415926535f                       // Pi constant
#define DIST_PER_PULSE (PI * WHEEL_DIAMETER_CM / PULSES_PER_WHEEL_REV)  // Distance per pulse in cm
#define WHEEL_BASE_CM 20.1f                   // Distance between wheels in cm (for turning calculations)

// --- Encoder Variables ---
volatile long EncCountA = 0;                   // Encoder count for Motor A (interrupt updated)
volatile long EncCountB = 0;                   // Encoder count for Motor B (interrupt updated)

// --- Distance / Speed Variables ---
float DistanceA = 0.0f;                        // Distance traveled by Motor A in cm
float DistanceB = 0.0f;                        // Distance traveled by Motor B in cm
float SpeedA = 0.0f;                           // Speed of Motor A in cm/s
float SpeedB = 0.0f;                           // Speed of Motor B in cm/s
float totaldistanceA = 0.0f;                   // Total distance traveled by Motor A
float totaldistanceB = 0.0f;                   // Total distance traveled by Motor B

// --- Global Sensor Variables ---
volatile int g_distance_ir = 999;              // IR sensor distance (cm)
volatile int g_cm_us1 = 999;                   // Ultrasonic sensor 1 distance (right side)
volatile int g_cm_us2 = 998;                   // Ultrasonic sensor 2 distance (left side)

// --- Motor Direction Variables ---
Direction dirA = CW;                           // Current direction of Motor A
Direction dirB = CW;                           // Current direction of Motor B

// --- Timer ---
mbed::Timer t;                                 // Timer for various timing operations
float prevTime = 0.0f;                         // Previous time reading
int shift_val = 0;                             // IR sensor shift value for calibration

// --- Robot State Variables (Odometry) ---
float robotX = 0.0f;                           // Robot X position in cm (0,0 is starting point)
float robotY = 0.0f;                           // Robot Y position in cm
float robotTheta = 0.0f;                       // Robot heading angle in radians (0 = facing north/forward)
long prevEncA = 0;                             // Previous encoder count for Motor A (for delta calculation)
long prevEncB = 0;                             // Previous encoder count for Motor B (for delta calculation)

// --- Threads ---
Thread sensor_thread;                          // Thread for sensor reading (runs independently)
Thread odometry_thread;                        // Thread for sensor reading (runs independently)
static float lastPrintTime = 0.0f;             // Last time debug info was printed

// --- Robot States (Finite State Machine) ---
enum RobotState {
  STATE_DRIVING,                               // Normal driving mode
  STATE_EVADING_BACKUP,                        // Backing up from obstacle
  STATE_FINISH_MAZE,                           //stop at 5
  STATE_EVADING_TURN                           // Turning to find clear path
};

RobotState g_robot_state = STATE_DRIVING;      // Current robot state
unsigned long g_evade_timer = 0;               // Timer for evade operations

// --- Pi Commands ---
static char g_command = 's';                   // Current movement command from Pi (f,b,l,r,s)
static int g_speed_int = 0;                    // Speed value from Pi (0-255)
static int g_angle_int = 86;                   // Steering angle from Pi (0-170, 86=straight) 86 because by this angle robot goes strait and difference between motor B and A will be componsated
static float g_speed_float = 0;                // Speed converted to 0.0-1.0 range

// --- Auto Scan Variables (every 40cm) ---
float lastScanX = 0.0f;                        // X position of last scan
float lastScanY = 0.0f;                        // Y position of last scan
bool scan40cmEnabled = true;                   // Enable/disable auto scan feature

// =============================================
// Global Variables for Path Storage and Maze Finish
// =============================================
#define MAX_PATH_POINTS 100                    // Maximum number of path points to store

struct PathPoint {
  float x;                                     // X coordinate of saved position
  float y;                                     // Y coordinate of saved position
  float theta;                                 // Robot heading angle at saved position
  bool isTurnPoint;                            // Whether this was a turning point
};

PathPoint pathHistory[MAX_PATH_POINTS];        // Array to store path history
int pathIndex = 0;                             // Index of last saved point
int pathSize = 0;                              // Total number of saved points
PathPoint newpath[MAX_PATH_POINTS];            // Buffer to store the calculated optimized path

bool aruco5_detected = false;                  // Flag: ArUco tag 5 detected by camera
bool maze_finished = false;                    // Flag: Maze finished (reached target)
unsigned long aruco5_detection_time = 0;       // Time when ArUco 5 was detected
bool return_mode = false;                      // Flag: Robot is in return mode (going back to start)

// =============================================
// Median Sorting Function
// =============================================
void sortArray(int a[], int n) {
  // Simple bubble sort for median filtering
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (a[j] > a[j + 1]) {
        int tmp = a[j];
        a[j] = a[j + 1];
        a[j + 1] = tmp;
      }
    }
  }
}

// =============================================
// Ultrasonic Sensor Reading Function
// =============================================
int readUltrasonic(int pin) {
  // Trigger ultrasonic pulse
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW); delayMicroseconds(2);
  digitalWrite(pin, HIGH); delayMicroseconds(15);
  digitalWrite(pin, LOW);
  
  // Switch to input mode and measure echo time
  pinMode(pin, INPUT);
  long duration = pulseIn(pin, HIGH, 25000);   // 25ms timeout
  
  if (duration == 0) return 999;              // No echo = no object
  
  int dist = uSecToCM(duration);              // Convert microseconds to cm
  if (dist == 0) return 999;                  // Invalid reading
  
  return dist;
}

// =============================================
// Sensor Thread Function (Runs in Background)
// =============================================
void sensor_thread_func() {
  byte high_byte, low_byte;
  int ir_readings[FILTER_SAMPLES];             // Array for IR sensor readings
  int us1_readings[FILTER_SAMPLES];            // Array for ultrasonic 1 (right) readings
  int us2_readings[FILTER_SAMPLES];            // Array for ultrasonic 2 (left) readings

  while (true) {
    // Collect multiple samples for each sensor
    for (int i = 0; i < FILTER_SAMPLES; i++) {
      // ---- IR Sensor Reading ----
      setI2CBus(0);                           // Select I2C bus 0
      Wire.beginTransmission(IR_SENSOR_ADDRESS);
      Wire.write(IR_SENSOR_DISTANCE_REG);     // Request distance register
      Wire.endTransmission();
      Wire.requestFrom(IR_SENSOR_ADDRESS, 2); // Request 2 bytes (distance data)

      long irStart = t.read_ms();
      bool irTimeout = false;
      
      // Wait for data with timeout
      while (Wire.available() < 2) {
        if (t.read_ms() - irStart > 40) { 
          irTimeout = true; 
          break; 
        }
      }

      if (!irTimeout) {
        high_byte = Wire.read();
        low_byte  = Wire.read();
        // Combine bytes and apply shift calibration
        ir_readings[i] = (high_byte * 16 + low_byte) / 16 / (int)pow(2, shift_val);
      } else {
        ir_readings[i] = 999;                  // Timeout = invalid reading
      }

      // ---- Ultrasonic Sensor Readings ----
      us1_readings[i] = readUltrasonic(USONIC1);  // Right sensor
      wait_us(20000);                          // 20ms delay between readings
      us2_readings[i] = readUltrasonic(USONIC2);  // Left sensor
    }

    // Apply median filter to remove noise
    sortArray(ir_readings, FILTER_SAMPLES);
    sortArray(us1_readings, FILTER_SAMPLES);
    sortArray(us2_readings, FILTER_SAMPLES);

    // Take middle value (median) as final reading
    g_distance_ir = ir_readings[FILTER_SAMPLES/2];
    g_cm_us1 = us1_readings[FILTER_SAMPLES/2];
    g_cm_us2 = us2_readings[FILTER_SAMPLES/2];

    wait_us(150000);                           // 150ms delay between sensor updates
  }
}

// =============================================
// Save Current Position to History
// =============================================
void saveCurrentPosition(bool isTurn = false) {
  // Circular buffer implementation
  if (pathIndex >= MAX_PATH_POINTS) {
    pathIndex = 0;                             // Wrap around if buffer full
  }
  
  // Store current position data
  pathHistory[pathIndex].x = robotX;
  pathHistory[pathIndex].y = robotY;
  pathHistory[pathIndex].theta = robotTheta;
  pathHistory[pathIndex].isTurnPoint = isTurn;
  
  pathIndex++;
  if (pathSize < MAX_PATH_POINTS) {
    pathSize++;                                // Increase size until buffer full
  }
  
  // Debug output
  Serial.print("[PATH] Saved position ");
  Serial.print(pathIndex - 1);
  Serial.print(": X=");
  Serial.print(robotX);
  Serial.print(" Y=");
  Serial.print(robotY);
  Serial.print(" Theta=");
  Serial.print(robotTheta * 180.0 / PI);
  Serial.print(" Turn=");
  Serial.println(isTurn ? "YES" : "NO");
}

// =============================================
// Setup Function (Runs Once at Startup)
// =============================================
void setup() {
  Serial.begin(9600);                          // USB serial for debugging
  Serial1.begin(115200);                       // Serial1 for communication with Raspberry Pi
  Wire.begin();                                // Initialize I2C bus

  // Configure motor control pins
  pinMode(MOTOR_DIRA, OUTPUT);
  pinMode(MOTOR_DIRB, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);
  
  // Configure encoder input pins
  pinMode(MOTOR_ENCA, INPUT);
  pinMode(MOTOR_ENCB, INPUT);

  // Attach encoder interrupt handlers
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), ENCA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), ENCB_ISR, RISING);

  t.start();                                   // Start timer
  prevTime = t.read();

  // Read IR sensor shift value for calibration
  setI2CBus(0);
  Wire.beginTransmission(IR_SENSOR_ADDRESS);
  Wire.write(IR_SENSOR_SHIFT_REG);
  Wire.endTransmission();
  Wire.requestFrom(IR_SENSOR_ADDRESS, 1);
  while (Wire.available() == 0);              // Wait for data
  shift_val = Wire.read();

  // Initialize path history array
  for(int i = 0; i < MAX_PATH_POINTS; i++) {
    pathHistory[i].x = 0;
    pathHistory[i].y = 0;
    pathHistory[i].theta = 0;
    pathHistory[i].isTurnPoint = false;
  }
    
  // Start sensor and odometry reading thread
  Serial.println("Starting sensor thread...");
  sensor_thread.start(sensor_thread_func);
  odometry_thread.start(updateOdometry);

  Serial1.println("Arduino: Ready");          // Tell Pi we're ready
  
  // Save initial position (starting point)
  saveCurrentPosition(false); 
}

// =============================================
//  Choose Side and Turn 90° (Obstacle Avoidance)
// =============================================
void chooseSideAndTurn90(float baseSpeed) {
  
  // Read ultrasonic sensors for obstacle detection
  int distR = (g_cm_us1 > 0 && g_cm_us1 < 900) ? g_cm_us1 : 0; // Right side
  int distL = (g_cm_us2 > 0 && g_cm_us2 < 900) ? g_cm_us2 : 0; // Left side

  Serial.print("[CHOICE] US1(R)=");
  Serial.print(distR);
  Serial.print(" cm, US2(L)=");
  Serial.print(distL);
  Serial.println(" cm");

  drive_stop();
  delay(120);                                  // Brief pause
  
  // Decision logic: turn toward more open space
  if (distR == 0 && distL == 0) {
    // No valid readings, default to left
    Serial.println("[CHOICE] No valid US readings, default LEFT");
    turn90degrees_left(baseSpeed);
    Serial.println("[CHOICE] Finished LEFT turn");
    return;
  }

  if (distR > distL) {
    // More space on right side
    Serial.println("[CHOICE] Turning RIGHT 90deg (more space on RIGHT).");
    turn90degrees_right(baseSpeed);
    Serial.println("[CHOICE] Finished RIGHT turn");
  } else {
    // More space on left side or equal
    Serial.println("[CHOICE] Turning LEFT 90deg (more space on LEFT).");
    turn90degrees_left(baseSpeed);
    Serial.println("[CHOICE] Finished LEFT turn");
  }

  delay(150);                                  // Pause after turn
  drive_forward_steer(baseSpeed, 86);          // Continue straight
  Serial.println("[CHOICE] Continuing straight after turn");
}

// =============================================
// Encoder Interrupt Service Routines
// =============================================
void ENCA_ISR() {  // Increment or decrement based on motor direction
  if (dirA == CW) EncCountA++; 
  else EncCountA--; 
}

void ENCB_ISR() { // Increment or decrement based on motor direction
    if (dirB == CCW) EncCountB++; 
  else EncCountB--; 
}

// =============================================
// Motor Control Functions
// =============================================
void motorSetDir(Motor m, Direction d) {
  // Set motor direction pin
  if (m == A) { 
    digitalWrite(MOTOR_DIRA, d); 
    dirA = d; 
  } else { 
    digitalWrite(MOTOR_DIRB, d); 
    dirB = d; 
  }
}

void drive(int motor, int dir, float speed) {
  // Control individual motor
  int pwm = constrain((int)(speed*255.0), 0, 255);  // Convert 0.0-1.0 to 0-255 PWM
  
  if (motor == 0) {
    motorSetDir(A, dir==0?CW:CCW);             // Set direction
    analogWrite(MOTOR_PWMA, pwm);              // Set speed
  } else {
    motorSetDir(B, dir==0?CCW:CW);             // Opposite direction convention for Motor B
    analogWrite(MOTOR_PWMB, pwm);
  }
}

void drive_forward(float s) { drive(0,0,s); drive(1,0,s); }       // Both motors forward
void drive_backward(float s){ drive(0,1,s); drive(1,1,s); }       // Both motors backward
void drive_stop() { drive(0,0,0); drive(1,0,0); }                 // Stop both motors
void drive_left(float s){ drive(0,1,s); drive(1,0,s); }           // Turn left in place
void drive_right(float s){ drive(0,0,s); drive(1,1,s); }          // Turn right in place

// =============================================
// Steering Functions (Differential Drive)
// =============================================
void drive_forward_steer(float baseSpeed, int angle) {
  // Convert angle (0-170) to turn ratio (-1.0 to 1.0)
  float tr = (angle - 90.0) / 90.0;
  float sA, sB;

  if (tr > 0) {
    // Turning right: left motor full speed, right motor reduced
    sA = baseSpeed;
    sB = baseSpeed*(1.0-tr);
  } else {
    // Turning left: right motor full speed, left motor reduced
    sA = baseSpeed*(1.0+tr);
    sB = baseSpeed;
  }

  drive(0,0,sA);   // Motor A
  drive(1,0,sB);   // Motor B
}

void drive_backward_steer(float baseSpeed, int angle) {
  // Same as forward but both motors reversed
  float tr = (angle - 90.0) / 90.0;
  float sA, sB;

  if (tr > 0) {
    sA = baseSpeed;
    sB = baseSpeed*(1.0-tr);
  } else {
    sA = baseSpeed*(1.0+tr);
    sB = baseSpeed;
  }

  drive(0,1,sA);   // Motor A backward
  drive(1,1,sB);   // Motor B backward
}

// =============================================
// Precise 90° Turning Functions
// =============================================
const float PI_BY_2 = PI / 2.0;
// Snap to the next Left/CCW 90 degree angle (Ceiling)
// Usage: When you want to correct alignment towards the left   ,  Ex: 50 deg -> 90 deg | -10 deg -> 0 deg
float get_snap_left_angle(float current_theta) {
  float target_theta = PI_BY_2 * ceil(current_theta / PI_BY_2);

  // Normalize
  while (target_theta > PI) target_theta -= 2 * PI;
  while (target_theta <= -PI) target_theta += 2 * PI;
  
  return target_theta;
}

// Snap to the next Right/CW 90 degree angle (Floor)
// Usage: When you want to correct alignment towards the right  ,  Ex: 50 deg -> 0 deg | -10 deg -> -90 deg
float get_snap_right_angle(float current_theta) {
  float target_theta = PI_BY_2 * floor(current_theta / PI_BY_2);

  // Normalize
  while (target_theta > PI) target_theta -= 2 * PI;
  while (target_theta <= -PI) target_theta += 2 * PI;
  
  return target_theta;
}

void turn90degrees_right(float baseSpeed) {
  Serial.println("[TURN] Starting RIGHT 90° turn");
  
  float target_theta = get_snap_right_angle(robotTheta-(PI/4));

  while (true) {
    float diff = target_theta - robotTheta;

    // Normalize difference for shortest path again
    while (diff > PI) diff -= 2 * PI;
    while (diff < -PI) diff += 2 * PI;

    if (abs(diff) * (180.0 / PI) < 1.0) {
      break;
    }

    Serial.println(diff);

    if (diff > 0) {
      drive_left(baseSpeed);
    } else {
      drive_right(baseSpeed);
    }
  }
  
  drive_stop();
  
  Serial.println("[TURN] Finished RIGHT 90° turn");
}

void turn90degrees_left(float baseSpeed) {

  float target_theta = get_snap_left_angle(robotTheta+(PI/4));

  while (true) {
    float diff = target_theta - robotTheta;

    // Normalize difference for shortest path again
    while (diff > PI) diff -= 2 * PI;
    while (diff < -PI) diff += 2 * PI;

    if (abs(diff) * (180.0 / PI) < 1.0) {
      break;
    }

    Serial.println(diff);

    if (diff > 0) {
      drive_left(baseSpeed);
    } else {
      drive_right(baseSpeed);
    }
  }
  
  drive_stop();
}

// =============================================
// Odometry Functions (Position Tracking)
// =============================================
void updateOdometry() {
  while(true){
  // Calculate encoder deltas since last update
  long deltaLeft = EncCountA - prevEncA;
  long deltaRight = EncCountB - prevEncB;
  
  // Convert pulses to distance
  float leftDistance = deltaLeft * DIST_PER_PULSE;
  float rightDistance = deltaRight * DIST_PER_PULSE;
  
  // Average distance (center of robot)
  float totalDistance = (leftDistance + rightDistance) / 2.0f;
  
  // Update position (non-standard coordinate system) Note: This uses Y as forward/backward, X as left/right
  robotY += totalDistance * cos(robotTheta);      // Forward/backward movement
  robotX += totalDistance * (-1)*sin(robotTheta); // Left/right movement
  
  // Calculate change in heading
  float deltaTheta = (rightDistance - leftDistance) / WHEEL_BASE_CM;  //error = real - command; +error = WHEEL_BASE_CM++; -error = WHEEL_BASE_CM--;
  
  robotTheta += deltaTheta;
  
  // Normalize angle to -π to π range
  if (robotTheta > PI) robotTheta -= 2 * PI;
  if (robotTheta < -PI) robotTheta += 2 * PI;
  
  // Save current encoder counts for next update
  prevEncA = EncCountA;
  prevEncB = EncCountB;
  wait_us(50000);
  }
}

// =============================================
// --- calibrate theta ---
// =============================================
int get_direction() {
  // Normalize angle to -180° to 180°
  if (robotTheta > PI) robotTheta -= 2 * PI;
  if (robotTheta < -PI) robotTheta += 2 * PI;
  
  float theta_deg = robotTheta * 180.0 / PI;

  int dir = 0;
  
  // Snap to 4 cardinal directions (simplifies navigation)
  if (theta_deg >= -45 && theta_deg < 45) {
        dir = 0;                  // North/forward
  }
  else if (theta_deg >= 45 && theta_deg < 135) {
            dir = 1;           // East/right
  }
  else if ((theta_deg >= 135 && theta_deg <= 180) || 
           (theta_deg >= -180 && theta_deg < -135)) {
           dir = 2;                     // South/backward
  }
  else if (theta_deg >= -135 && theta_deg < -45) {
             dir = 3;           // West/left
  } // dir : 0->N , 1->E , 2->s, 3->W

  return dir;
}

// =============================================
// Search Function (360° Rotation Scan)
// =============================================
void search() {
  float baseSpeed = g_speed_float;

  saveCurrentPosition(true);                    // Mark as turn point
  
  // Scan 4 directions (0°, 90°, 180°, 270°)
  Serial1.println("DIRECTION:0");               // Tell Pi current direction
  delay(500);                                   // Wait for Pi to detect and store
  
  turn90degrees_right(baseSpeed);               // Turn to 90°
  drive_stop();
  delay(2000);                                  // Wait for camera detection
  Serial1.println("DIRECTION:90");
  delay(500);

  turn90degrees_right(baseSpeed);               // Turn to 180°
  drive_stop();
  delay(2000);
  Serial1.println("DIRECTION:180");
  delay(500);

  turn90degrees_right(baseSpeed);               // Turn to 270°
  drive_stop();
  delay(2000);
  Serial1.println("DIRECTION:270");
  delay(500);
  
  turn90degrees_right(baseSpeed);               // Return to 0° (original direction)
  drive_stop();
  delay(2000);
  Serial1.println("DIRECTION:0");
  delay(500);

  lastScanX = robotX;
  lastScanY = robotY;
}

// =============================================
// Choose Degree Function (Direction Decision)  // return 0 if no turn else 1
// =============================================
int choose_degree() { 
  bool obstacle_front = (g_distance_ir < 5.5 && g_distance_ir > 0);
  
  if (obstacle_front) {
    // Can't go forward, go back to obstacle avoidance
    Serial.println("!!! Front obstacle in choose_degree!");
    g_robot_state = STATE_EVADING_BACKUP; 
    g_evade_timer = t.read_ms(); 
    return 0;
  }
  
  // Ask Pi to decide based on camera data
  Serial1.println("DECIDE");
  String decision = "";
  unsigned long startWait = millis();
  
  // Wait for Pi's decision (2 second timeout)
  while (decision == "" && (millis() - startWait < 2000)) {
    if (Serial1.available() > 0) {
      decision = Serial1.readStringUntil('\n');
      decision.trim();
    }
    delay(10);
  }
  
  if (decision != "" && decision != "NO_TAGS") {
    // Pi provided decision based on ArUco tags
    Serial.print("[ARUCO Decision] ");
    Serial.println(decision);
    
    float baseSpeed = g_speed_float;
    if (decision == "n90") {
      turn90degrees_right(baseSpeed);           // Turn right 90°
    } else if (decision == "n180") {
      turn90degrees_left(baseSpeed);            // Turn 180° (two left turns)
      turn90degrees_left(baseSpeed);
    } else if (decision == "n270") {
      turn90degrees_left(baseSpeed);            // Turn left 90°
    } else if(decision == "n0") {
      // Go straight (no turn needed)
      return 0;
    }
    delay(200);
    return 1;
  }
  
  // Fallback: Use ultrasonic sensors if no camera data
  Serial.println("[FALLBACK] No ArUco tags found, using ultrasonic decision");
  
  int distR = (g_cm_us1 > 0 && g_cm_us1 < 900) ? g_cm_us1 : 0;
  int distL = (g_cm_us2 > 0 && g_cm_us2 < 900) ? g_cm_us2 : 0;
  
  Serial.print("[ULTRASONIC] R=");
  Serial.print(distR);
  Serial.print("cm, L=");
  Serial.print(distL);
  Serial.println("cm");
  
  drive_stop();
  delay(200);
  
  if (distR == 0 && distL == 0) {
    // No valid readings, default left
    Serial.println("[ULTRASONIC] Default: LEFT");
    turn90degrees_left(g_speed_float);
  } else if (distR > distL) {
    // More space on right
    Serial.println("[ULTRASONIC] Turning RIGHT (more space)");
    turn90degrees_right(g_speed_float);
  } else {
    // More space on left or equal
    Serial.println("[ULTRASONIC] Turning LEFT (more space)");
    turn90degrees_left(g_speed_float);
  }
  
  saveCurrentPosition(true);                    // Mark as turn point
  delay(200);
}

// function to go to point
void go_to_point(float end_x, float end_y) {

  // --- 1. ROTATE TO TARGET ---
  float dx = end_x - robotX;
  float dy = end_y - robotY;
  if ( abs(dx) < 5) dx = 0 ; // reduce drifts 
  if ( abs(dy) < 5) dy = 0 ; // reduce drifts 
  // Calculate target angle
  float target_theta = atan2(dy, dx) - (PI / 2);

  // Normalize target_theta to -PI to PI
  if (target_theta > PI) target_theta -= 2 * PI;
  if (target_theta < -PI) target_theta += 2 * PI;

  float speed = 0.5;

  // loop until error is less than 1 degree
  while (true) {
    // A. Calculate the difference (error)
    float diff = target_theta - robotTheta;

    // B. Normalize difference to shortest path (-PI to PI)
    // This transforms 270 degrees into -90 degrees
    while (diff > PI) diff -= 2 * PI;
    while (diff < -PI) diff += 2 * PI;

    // C. Break if close enough (1 degree tolerance)
    if (abs(diff) * (180.0 / PI) < 1.0) {
      break;
    }

    // D. Turn based on the sign of the difference
    // If diff is positive, target is to the left. If negative, to the right.
    if (diff > 0) {
      drive_left(speed);
    } else {
      drive_right(speed);
    }
  }
  
  drive_stop(); // Good practice to stop briefly before moving forward
  delay(20);

  // --- 2. DRIVE FORWARD ---
  float target_dist = sqrt(dx * dx + dy * dy);
  float start_x = robotX;
  float start_y = robotY;

  while (sqrt(pow(robotX - start_x, 2) + pow(robotY - start_y, 2)) <= target_dist) {
    // You can keep your existing distance print logic here
    drive_forward_steer(speed,86);
  }
  normalize_theta();
  drive_stop();
  delay(20);
}

// --- calibrate theta ---
void normalize_theta() {
  // Normalize angle to -180° to 180°
  if (robotTheta > PI) robotTheta -= 2 * PI;
  if (robotTheta < -PI) robotTheta += 2 * PI;
  
  float theta_deg = robotTheta * 180.0 / PI;
  
  // Snap to 4 cardinal directions (simplifies navigation)
  if (theta_deg >= -45 && theta_deg < 45) {
    robotTheta = 0.0f;                          // North/forward
  }
  else if (theta_deg >= 45 && theta_deg < 135) {
    robotTheta = PI / 2.0f;                     // East/right
  }
  else if ((theta_deg >= 135 && theta_deg <= 180) || 
           (theta_deg >= -180 && theta_deg < -135)) {
    robotTheta = PI;                            // South/backward
  }
  else if (theta_deg >= -135 && theta_deg < -45) {
    robotTheta = -PI / 2.0f;                    // West/left
  }
}

// ==========================================
// ===  Loop Closure / Path Optimization  ===
// ==========================================
void loop_go_to_home() {
  // start from last saved point (Start backwards)
  int i = pathIndex - 1; 
  while (i >= 0) {
    
    // --- Loop Optimization Check ---
    for (int k = i - 1; k >= 0; k--) {
      float dx = pathHistory[i].x - pathHistory[k].x;
      float dy = pathHistory[i].y - pathHistory[k].y;
      float dist = sqrt(dx*dx + dy*dy);

      // if distance was less than 15cm --> Loop!
      if (dist < 15.0) {
        i = k; // JUMP: Index 'i' --> 'k'
        break; // Break for removing the bigest loop 
      }
    }
    go_to_point(pathHistory[i].x, pathHistory[i].y);
    delay(50);
    // Move to the next point (backwards towards 0)
    i--;
  }
  drive_stop();
  // Reset index
  pathIndex = 0;
  pathSize = 0; 
}

void loop() {
  // --- Read new Pi command ---
  if (Serial1.available() > 0) {
    String msg = Serial1.readStringUntil('\n');
    msg.trim();
    if (msg.length() > 0) {
      if (msg.startsWith("GOTO:")) {
          // Parse: GOTO:x,y,theta
          int comma1 = msg.indexOf(',', 5);  // After GOTO:
          int comma2 = msg.indexOf(',', comma1+1);
          if (comma1 > 0 && comma2 > 0) {
              float tx = msg.substring(5, comma1).toFloat();
              float ty = msg.substring(comma1+1, comma2).toFloat();
              go_to_point(tx, ty);
          }
            return;  // After GOTO, don't continue loop (to prevent parsing cmd)
      }

      // Parse cmd/spd/ang for other commands
      char cmd = 's';
      int spd = 0;
      int ang = 86;
      int sp1 = msg.indexOf(' ');
      if (sp1 == -1) { 
        cmd = msg.charAt(0); 
      }
      else {
        cmd = msg.substring(0, sp1).charAt(0);
        int sp2 = msg.indexOf(' ', sp1+1);
        if (sp2 == -1) {
          spd = msg.substring(sp1+1).toInt();
        } else {
          spd = msg.substring(sp1+1, sp2).toInt();
          ang = msg.substring(sp2+1).toInt();
        }
      }

      if(cmd == 'e'){
        if(spd == 127 && ang == 86)
          aruco5_detected = true;
          
        else
        aruco5_detected = false;
      }

      if(cmd == 'h'){  // with command h from python robot will get back
        loop_go_to_home();
      }

      g_command = cmd;
      g_speed_int = spd;
      g_angle_int = ang;
      g_speed_float = constrain(g_speed_int,0,255) / 255.0;

      // Reset theta when forward command is received
      if (g_command == 'f') {
        robotTheta = 0.0f;
        Serial.println("[ODOM] Reset theta to 0 for forward movement");
      }
    }
  }

  // --- Detect obstacles for 180 degree turn ---
  if (g_distance_ir < 15 && g_cm_us2 < 20 && g_cm_us1 < 20 && g_distance_ir > 0) {
    drive_stop();
    delay(200);
    drive_backward(0.7);
    delay(500);
    // Return to normal state
    g_robot_state = STATE_EVADING_BACKUP;
  }  
  
  // --- Detect obstacles ---
  bool obstacle_front = (g_distance_ir < 5.5 && g_distance_ir > 0);

  float lastdistance = sqrt( (lastScanX-robotX)*(lastScanX-robotX) + (lastScanY-robotY)*(lastScanY-robotY) );

  // --- State Machine ---
  switch (g_robot_state) {
    case STATE_FINISH_MAZE: drive_stop(); break;
    case STATE_DRIVING:
      if (g_command == 'f') {
        if (obstacle_front || lastdistance > 60) {
          // REACTION: Front obstacle or 40cm movement! Starting evade operation...
          g_robot_state = STATE_EVADING_BACKUP; 
          g_evade_timer = t.read_ms(); 
        }
        else {
          drive_forward_steer(g_speed_float, g_angle_int); 
        }
      } else {
          // Execute non-forward commands        
        switch (g_command) {
          case 'b': drive_backward_steer(g_speed_float, g_angle_int); break;
          case 'l': drive_left(g_speed_float); break;
          case 'r': drive_right(g_speed_float); break;
          case 's': default: drive_stop(); break;
        }
      }
      break; 
    
      // --- State 2: Backing up (ignoring Pi) ---    
    case STATE_EVADING_BACKUP:
      if (t.read_ms() - g_evade_timer < 600) { 
        // dir : 0->N , 1->E , 2->s, 3-W
        int dir = get_direction();    // customize backwards for each direction
          if (dir == 0){              drive_backward(0.2); }
          else if (dir == 3){         drive_backward(0.1); }
          else if(dir == 1){          drive_backward(0.7); }
          else{
          drive_backward(0.15); 
          }
      } else {
        Serial.println("... Backing up finished.");
        g_robot_state = STATE_DRIVING;  // back to normal move
        if(!aruco5_detected){
          search();
          if(!choose_degree())              // if didn't choose by arucos and python 
            chooseSideAndTurn90(0.5);       // choose direction by ultrasonics 
            g_robot_state = STATE_DRIVING;
        }else{
          g_robot_state = STATE_FINISH_MAZE;  
        }
      }
      break; 
  }

  // ---------- Encoder & Speed (print every 200ms) ----------
  float currentTime = t.read();
  float elapsedTime = currentTime - prevTime;
  if (elapsedTime <= 0) elapsedTime = 0.001f;

  DistanceA = EncCountA * DIST_PER_PULSE;
  DistanceB = EncCountB * DIST_PER_PULSE;

  SpeedA = (EncCountA * DIST_PER_PULSE) / elapsedTime;
  SpeedB = (EncCountB * DIST_PER_PULSE) / elapsedTime;
  
  if (currentTime - lastPrintTime > 0.2f) {
    Serial.println("------------------------");
    Serial.print("[SENSORS] IR: ");
    Serial.print(g_distance_ir);
    Serial.print(" cm | US1(R): ");
    Serial.print(g_cm_us1);
    Serial.print(" cm | US2(L): ");
    Serial.print(g_cm_us2);
    Serial.print(" cm | DistanceMoved: ");
    Serial.println(" cm");
    Serial.print("[ENC] Motor A -> Pulses: "); Serial.print(EncCountA);
    Serial.print(", Speed: "); Serial.print(SpeedA); Serial.print(" cm/s");
    Serial.print(", Distance: "); Serial.print(DistanceA);
    Serial.print(", total A: "); totaldistanceA += DistanceA; Serial.print(totaldistanceA);
    Serial.println(" cm");
    Serial.print("[ENC] Motor B -> Pulses: "); Serial.print(EncCountB);
    Serial.print(", Speed: "); Serial.print(SpeedB); Serial.print(" cm/s");
    Serial.print(", Distance: "); Serial.print(DistanceB);
    Serial.print(", total B: "); totaldistanceB += DistanceB; Serial.print(totaldistanceB);
    Serial.println(" cm");
    Serial.print("[Position] X=");
    Serial.print(robotX);
    Serial.print("  Y=");
    Serial.print(robotY);
    Serial.print("  TH=");
    Serial.print(robotTheta * 180.0 / PI);
    Serial.print("  LastScanX=");
    Serial.print(lastScanX);
    Serial.print("  LastScanY=");
    Serial.println(lastScanY);
    
    // --- Send sensor and odometry data to Pi ---    
    Serial1.print("IR:");
    Serial1.print(g_distance_ir);
    Serial1.print("|US1:");
    Serial1.print(g_cm_us1);
    Serial1.print("|US2:");
    Serial1.print(g_cm_us2);
    Serial1.print("|ODOM:");
    Serial1.print(robotX); Serial1.print(",");
    Serial1.print(robotY); Serial1.print(",");    
    Serial1.print(robotTheta * 180.0 / PI); // Convert to degrees
    Serial1.print("|SCAN_INFO:");
    Serial1.print(lastScanX);          Serial1.print(",");
    Serial1.print(robotX - lastScanX); Serial1.print(",");
    Serial1.print(lastScanY);          Serial1.print(",");
    Serial1.print(robotY - lastScanY); Serial1.print(",");
    Serial1.println();

    lastPrintTime = currentTime;
  }

  prevTime = currentTime;
  wait_us(10000);     // 10ms
}