import cv2
import time
import numpy as np
from picamera2 import Picamera2
import serial
import cv2.aruco as aruco
import threading
import math
import json
from datetime import datetime
from collections import deque
import matplotlib.pyplot as plt

goto_done = False       # Global flag for GOTO_DONE - used to signal when a GOTO command is completed

# ===============================
# === 1. Arduino Serial Setup ===
# ===============================

try:
    # Open serial communication with Arduino at 115200 baud rate
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    time.sleep(2)   # Wait for Arduino to initialize
    print(" Serial port opened. Ready for commands.")
except Exception as e:
    print(f" Error opening serial port: {e}")
    exit()

# =========================
# === 2. Camera Setup ===
# =========================

# Initialize Raspberry Pi camera
cam = Picamera2()
# Configure camera for RGB format with 800x600 resolution
config = cam.create_preview_configuration(main={'format': 'RGB888', 'size': (800, 600)})
cam.configure(config)
cam.start()         # Start camera capture
time.sleep(0.1)     # Allow camera to warm up

# Setup aruco dictionary and parameters for marker detection
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)   # Use 6x6 ArUco markers
aruco_parameters = aruco.DetectorParameters_create()

# =========================
# === 3. Color HSV Ranges ===
# =========================

# Define HSV color ranges for different colors
COLOR_RANGES = {
    "Red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "Blue": [((100, 150, 20), (140, 255, 255))],
    "Green": [((35, 75, 0), (85, 255, 255))],
    "Purple": [((125, 40, 30), (165, 255, 255))]
}

kernel = np.ones((3, 3), np.uint8)  # Kernel for morphological operations (noise removal) 

# Colors for text display on shapes
text_colors = {
    "Red": (0, 0, 255),  # Red in BGR     Blue=0, Green=0, Red=255
    "Blue": (255, 0, 0),                # Blue=255, Green=0, Red=0
    "Green": (0, 255, 0),               # Blue=0, Green=255, Red=0
    "Purple": (128, 0, 128)             # Blue=128, Green=0, Red=128
}

# =========================
# === 4. Shape Detection Function ===
# =========================

def identify_shape(contour):
    """Identify the shape from contour"""
    perimeter = cv2.arcLength(contour, True)                        # Calculate perimeter of contour
    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)      # Approximate contour to polygon
    vertices = len(approx)                                          # Number of vertices
    area = cv2.contourArea(contour)                                 # Area of contour

     # Filter out small contours
    if area < 400:
        return "Unknown", approx

    # Triangle (3 vertices)
    if vertices == 3:
        return "Triangle", approx

    # Rectangle (4 vertices)
    elif vertices == 4:
        x, y, w, h = cv2.boundingRect(contour)
        ratio = w / float(h)
        return ("Rectangle" if 0.5 < ratio < 2 else "Irregular-4sides"), approx

    # 5-point star (approx 8–12 vertices)
    elif 8 <= vertices <= 12:
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        if hull_area > 0 and (area / hull_area) < 0.9:              
            return "5-Point Star", approx
        else:
            return "Circle", approx

    # Circle check
    else:
        x, y, w, h = cv2.boundingRect(contour)
        ratio = w / float(h)
        # Check if contour is circular (aspect ratio close to 1, area fills bounding box)
        if 0.8 < ratio < 1.2 and area / (w * h) > 0.6:
            return "Circle", approx 

    return "Unknown", approx

# =========================
# === 5. Data Structures ===
# =========================

# Store ArUco IDs in different directions
direction_data = {
    "n0": [],     # Direction 0 degrees     (forward or North)
    "n90": [],    # Direction 90 degrees    (right or East)
    "n180": [],   # Direction 180 degrees   (backward or South)
    "n270": []    # Direction 270 degrees   (left or West)
}

current_direction = "n0"  # Current direction
waiting_for_scan = False  # Are we waiting for scan?
scan_complete = False     # Is scan complete?
at_aruco_point = False    # Have we reached ArUco 5?

# Odometry data from Arduino
odom_data = {
    "x": 0.0,      # X position in cm
    "y": 0.0,      # Y position in cm
    "theta": 0.0   # Heading angle in degrees
}

# Sensor data from Arduino
sensor_data = {
    "ir": 999,   # IR sensor distance in cm (front)
    "us1": 999,  # Ultrasonic sensor 1 distance in cm (right)
    "us2": 999   # Ultrasonic sensor 2 distance in cm (left)
}

# Scan information from Arduino
scan_info = {
    "last_scan_x": 0.0,     # X position of last scan
    "last_scan_y": 0.0,     # Y position of last scan
    "scan_enabled": True    # Flag indicating if auto-scan is enabled
}

# Robot path history (received from Arduino)
robot_path_history = deque()  # Using deque for efficient append/pop operations
retrace_in_progress = False   # Flag indicating if robot is retracing path
mission_complete = False      # Flag indicating if mission is complete
aruco_tag_5_detected = False  # Flag indicating if ArUco tag 5 is detected

# =========================
# === 6. Path History for Retrace ===
# =========================
class PathHistory:
    """Class to store and manage path history"""
    def __init__(self, max_points=100):
        """Initialize path history with maximum number of points"""
        self.max_points = max_points
        self.points = []                        # List to store path points
        self.save_file = "path_history.json"    # File to save/load path history
        self.load_history()                     # Load existing history if available

    def add_point(self, x, y, theta, scan_point=False, sensor_data=None):
        """Add a point to path history"""
        point = {
            "x": x,
            "y": y,
            "theta": theta,
            "scan_point": scan_point,                          # Whether this is a scan/turn point
            "timestamp": datetime.now().strftime("%H:%M:%S"),  # Time of point
            "sensor_data": sensor_data if sensor_data else {}  # Sensor readings
        }
        self.points.append(point)
        # Limit number of points to prevent memory issues
        if len(self.points) > self.max_points:
            self.points = self.points[-self.max_points:]
        print(f" Path point added: X={x:.1f}, Y={y:.1f}, theta={theta:.1f}, Scan={scan_point}")
        self.save_history()                                     # Save after adding

    def clear(self):
        """Clear all path points"""
        self.points = []
        print(" Path history cleared")
        self.save_history()

    def save_history(self):
        """Save path history to file"""
        try:
            with open(self.save_file, 'w') as f:
                json.dump(self.points, f, indent=2)
        except Exception as e:
            print(f" Error saving path history: {e}")

    def load_history(self):
        """Load path history from file"""
        try:
            with open(self.save_file, 'r') as f:
                self.points = json.load(f)
            print(f" Loaded {len(self.points)} path points from history")
        except FileNotFoundError:
            print(" No previous path history found")
        except Exception as e:
            print(f" Error loading path history: {e}")

    def print_history(self):
        """Print all path points"""
        print("\n" + "="*50)
        print("PATH HISTORY")
        print("="*50)
        for i, point in enumerate(self.points):
            print(f"{i:3d}. X={point['x']:6.1f}, Y={point['y']:6.1f}, "
                  f"theta={point['theta']:6.1f}degree, Scan={'y' if point['scan_point'] else 'n'}, "
                  f"Time={point['timestamp']}")
        print("="*50)

    def get_points(self):
        """Get all points"""
        return self.points

    def get_last_point(self):
        """Get last point"""
        return self.points[-1] if self.points else None

    def get_point_count(self):
        """Get number of points"""
        return len(self.points)

    def get_reverse_path(self):
        """Get reverse path for return journey"""
        return list(reversed(self.points))

# Create path history object
path_history = PathHistory(max_points=9999999999999999)

# =========================
# === 7. Serial Reading Thread ===
# =========================

def serial_reader():
    """Thread for reading serial data from Arduino"""
    global waiting_for_scan, current_direction, odom_data, sensor_data, scan_info
    global at_aruco_point, mission_complete, aruco_tag_5_detected, retrace_in_progress
    global goto_done

    while True:
        if ser.in_waiting > 0:
            try:
                # Read line from Arduino
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"Arduino: {line}")
                
                # =============================================
                # Special messages from Arduino
                # =============================================
                # If reached ArUco Tag 5
                if line == "AT_ARUCO_TAG5":
                    print(" Robot reached ArUco Tag 5! ")
                    print("IR < 10cm - Mission completed!")
                    at_aruco_point = True
                    aruco_tag_5_detected = True
                    mission_complete = True
                    # Save final position
                    path_history.add_point(odom_data['x'], odom_data['y'], odom_data['theta'], scan_point=True, sensor_data=sensor_data.copy())
                    print(" Robot stopped. Waiting for return command...")
                    print(" Press 'h' key to turn and return home.")
                    continue

                # If DIRECTION command received
                if line.startswith("DIRECTION:"):
                    direction = line.split(":")[1]
                    if direction in ["0", "90", "180", "270"]:
                        dir_key = f"n{direction}"
                        current_direction = dir_key
                        waiting_for_scan = True
                        print(f" Starting scan in direction: {dir_key}")
                        
                # If decision request received
                elif line == "DECIDE":
                    print(" Arduino requested decision")
                    # Process data and select best direction
                    best_direction = analyze_and_decide()
                    # Send result to Arduino
                    ser.write(f"{best_direction}\n".encode())
                    print(f" Decision sent: {best_direction}")
                    scan_complete = True
                
                # Parse sensor and odometry data
                elif "|ODOM:" in line and "|SCAN_INFO:" in line:
                    try:
                        # Extract different parts
                        parts = line.split("|")
                        
                        for part in parts:
                            # IR section
                            if part.startswith("IR:"):
                                try:
                                    sensor_data["ir"] = int(part.split(":")[1])
                                except:
                                    pass
                            
                            # US1 section (right ultrasonic)
                            elif part.startswith("US1:"):
                                try:
                                    sensor_data["us1"] = int(part.split(":")[1])
                                except:
                                    pass
                            
                            # US2 section (left ultrasonic)
                            elif part.startswith("US2:"):
                                try:
                                    sensor_data["us2"] = int(part.split(":")[1])
                                except:
                                    pass
                            
                            # Odometry data (position and angle)
                            elif part.startswith("ODOM:"):
                                try:
                                    odom_part = part.split(":")[1]
                                    x_str, y_str, theta_str = odom_part.split(",")
                                    odom_data["x"] = float(x_str)
                                    odom_data["y"] = float(y_str)
                                    odom_data["theta"] = float(theta_str)
                                    # Store in robot path history
                                    if len(robot_path_history) == 0 or \
                                       abs(robot_path_history[-1][0] - odom_data['x']) > 1.0 or \
                                       abs(robot_path_history[-1][1] - odom_data['y']) > 1.0:
                                        robot_path_history.append((odom_data['x'], odom_data['y'], odom_data['theta']))
                                except:
                                    pass
                            
                            # SCAN_INFO section
                            elif part.startswith("SCAN_INFO:"):
                                try:
                                    scan_part = part.split(":")[1]
                                    scan_parts = scan_part.split(",")
                                    if len(scan_parts) >= 5:
                                        scan_info["last_scan_x"] = float(scan_parts[0])
                                        scan_info["dist_x"] = float(scan_parts[1])
                                        scan_info["last_scan_y"] = float(scan_parts[2])
                                        scan_info["dist_y"] = float(scan_parts[3])
                                        scan_info["scan_enabled"] = (scan_parts[4] == "1")
                                except:
                                    pass
                                    
                    except Exception as e:
                        print(f" Error processing data: {e}, Line: {line}")
                
                # Process simple sensor lines
                elif "IR:" in line and "US1(R):" in line and "US2(L):" in line:
                    try:
                        # Extract values
                        ir_part = line.split("IR:")[1].split("cm")[0].strip()
                        us1_part = line.split("US1(R):")[1].split("cm")[0].strip()
                        us2_part = line.split("US2(L):")[1].split("cm")[0].strip()
                        
                        sensor_data["ir"] = int(float(ir_part))
                        sensor_data["us1"] = int(float(us1_part))
                        sensor_data["us2"] = int(float(us2_part))
                    except:
                        pass
                
                # PATH_POINT messages from Arduino
                elif line.startswith("PATH_POINT:"):
                    try:
                        data = line.split("PATH_POINT:")[1]
                        x_str, y_str, theta_str, scan_str = data.split(",")
                        x = float(x_str)
                        y = float(y_str)
                        theta = float(theta_str)
                        scan_point = (scan_str == "1")
                        # Store in history
                        path_history.add_point(x, y, theta, scan_point, sensor_data.copy())
                    except Exception as e:
                        print(f" Error processing PATH_POINT: {e}")

                # Handle GOTO_DONE
                if line == "GOTO_DONE":
                    goto_done = True
                    print(" Received GOTO_DONE from Arduino - point reached!")

            except Exception as e:
                print(f" Error reading serial: {e}")

# =========================
# === 8. Analysis Functions ===
# =========================

def analyze_and_decide():
    """Analyze stored data and select best direction"""
    print("\n Analyzing collected data:")
    
    # Check if no tags found
    total_ids = 0
    for direction, ids in direction_data.items():
        if ids:
            unique_ids = list(set(ids))
            print(f"  {direction}: {len(unique_ids)} tags - IDs = {sorted(unique_ids)}")
            total_ids += len(unique_ids)
        else:
            print(f"  {direction}: No IDs found")
    
    # If no tags found
    if total_ids == 0:
        print(" No ArUco IDs found. Sending NO_TAGS")
        # Save current position in history
        path_history.add_point(odom_data['x'], odom_data['y'], odom_data['theta'], scan_point=True, sensor_data=sensor_data.copy())
        # Clear data for next scan
        for direction in direction_data:
            direction_data[direction].clear()
        return "NO_TAGS"
    
    # Find maximum ID in each direction
    max_ids = {}
    for direction, ids in direction_data.items():
        if ids:
            max_ids[direction] = max(ids)
        else:
            max_ids[direction] = -1
    
    print(f"\n Maximum ID in each direction: {max_ids}")
    
    # Select direction with highest ID
    best_direction = max(max_ids, key=max_ids.get)
    
    # If all values are -1 (no IDs found)
    if max_ids[best_direction] == -1:
        print(" All values are -1. Sending NO_TAGS")
        # Save current position in history
        path_history.add_point(odom_data['x'], odom_data['y'], odom_data['theta'], scan_point=True, sensor_data=sensor_data.copy())
        # Clear data for next scan
        for direction in direction_data:
            direction_data[direction].clear()
        return "NO_TAGS"
    else:
        print(f" Best direction selected: {best_direction} (highest ID: {max_ids[best_direction]})")
    
    # Save current position in history (decision point)
    path_history.add_point(odom_data['x'], odom_data['y'], odom_data['theta'], scan_point=True, sensor_data=sensor_data.copy())
    
    # Clear data for next scan
    for direction in direction_data:
        direction_data[direction].clear()
    
    return best_direction

# =========================
# === 9. Function to Draw Robot Position ===
# =========================

def draw_robot_position(image, x, y, theta, scale=0.25, offset_x=150, offset_y=150):
    """
    Display robot position on image for 400x300cm map
    """
    # Map 400x300cm
    map_width_cm = 220  # Map width in cm
    map_height_cm = 160  # Map height in cm
    panel_size_px = 300  # Panel size in pixels
    
    # Calculate scale to display full map in panel
    new_scale_x = panel_size_px / map_width_cm  # Scale for X axis
    new_scale_y = panel_size_px / map_height_cm  # Scale for Y axis
    
    # Use smaller scale to ensure full display
    new_scale = min(new_scale_x, new_scale_y) * 0.9  # 90% for margin
    
    # Convert cm coordinates to pixels
    # Coordinate 0,0 at panel center
    center_x = panel_size_px // 2
    center_y = panel_size_px - 30
    
    # Conversion: X to right positive, Y to up positive (in map)
    # In image Y to down is positive, so we negate
    px = int(center_x + x * new_scale)
    py = int(center_y - y * new_scale)  # Negative because Y axis in image is inverted
    
    # Draw map boundary (400x300cm)
    map_left = int(center_x - 80 * new_scale)
    map_right = int(center_x + 80 * new_scale)
    map_top = int(center_y - 200 * new_scale)
    map_bottom = int(center_y + 10 * new_scale)
    
    # Draw map border
    cv2.rectangle(image, (map_left, map_top), (map_right, map_bottom), (100, 100, 100), 2)
    
    # Draw robot position point
    if mission_complete:
        cv2.circle(image, (px, py), 8, (0, 0, 255), -1)  # Red circle for mission complete
    else:
        cv2.circle(image, (px, py), 6, (0, 255, 0), -1)  # Green circle
    
    # Draw robot direction (arrow)
    arrow_length = 15
    angle_rad = np.radians(theta)
    end_x = int(px + arrow_length * np.sin(angle_rad))
    end_y = int(py - arrow_length * np.cos(angle_rad))  # Negative because Y axis is inverted
    
    cv2.arrowedLine(image, (px, py), (end_x, end_y), (255, 0, 0), 2)
    
    # Draw last scan position
    if scan_info["last_scan_x"] != 0 or scan_info["last_scan_y"] != 0:
        scan_px = int(center_x + scan_info["last_scan_x"] * new_scale)
        scan_py = int(center_y - scan_info["last_scan_y"] * new_scale)
        cv2.circle(image, (scan_px, scan_py), 5, (255, 0, 255), 2)  # Purple circle
    
    # Draw path history from Arduino
    if len(robot_path_history) > 1:
        for i in range(1, len(robot_path_history)):
            prev_point = robot_path_history[i-1]
            curr_point = robot_path_history[i]
            prev_px = int(center_x + prev_point[0] * new_scale)
            prev_py = int(center_y - prev_point[1] * new_scale)
            curr_px = int(center_x + curr_point[0] * new_scale)
            curr_py = int(center_y - curr_point[1] * new_scale)
            # Line color: Blue for forward path
            cv2.line(image, (prev_px, prev_py), (curr_px, curr_py), (255, 255, 0), 1)
    
    # If in return mode, draw return path
    if retrace_in_progress:
        reverse_path = path_history.get_reverse_path()
        if len(reverse_path) > 1:
            for i in range(len(reverse_path)-1):
                prev_point = reverse_path[i]
                curr_point = reverse_path[i+1]
                prev_px = int(center_x + prev_point["x"] * new_scale)
                prev_py = int(center_y - prev_point["y"] * new_scale)
                curr_px = int(center_x + curr_point["x"] * new_scale)
                curr_py = int(center_y - curr_point[1] * new_scale)
                # Line color: Red for return path
                cv2.line(image, (prev_px, prev_py), (curr_px, curr_py), (0, 0, 255), 1)
    
    # Draw coordinate grid (every 5cm)
    grid_spacing_cm = 5
    
    # Vertical lines (X axis)
    for i in range(-80, 81, grid_spacing_cm):
        x_pos = int(center_x + i * new_scale)
        cv2.line(image, (x_pos, map_top), (x_pos, map_bottom), (80, 80, 80), 1)
        # X value labels
        if i in [-80, -60, -40, -20, 0, 20, 40, 60, 80]:
            cv2.putText(image, f"{i}", (x_pos - 10, map_bottom + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)
    
    # Horizontal lines (Y axis)
    for i in range(-10, 201, grid_spacing_cm):
        y_pos = int(center_y - i * new_scale)  # Negative because Y is inverted
        cv2.line(image, (map_left, y_pos), (map_right, y_pos), (80, 80, 80), 1)
        # Y value labels - only specific values (every 20cm)
        y_label_values = [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200]
        for i in y_label_values:
            y_pos = int(center_y - i * new_scale)
            cv2.putText(image, f"{i}", (map_left - 25, y_pos + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (150, 150, 150), 1)    
    # Main axes
    cv2.line(image, (map_left, center_y), (map_right, center_y), (255, 255, 255), 1)  # X axis
    cv2.line(image, (center_x, map_top), (center_x, map_bottom), (255, 255, 255), 1)  # Y axis
    
    # Axis labels
    cv2.putText(image, "X", (map_right - 10, center_y + 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(image, "Y", (center_x - 15, map_top + 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Display zero at center
    cv2.putText(image, "0", (center_x - 8, center_y + 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    
    # Display home (starting point) at center
    cv2.circle(image, (center_x, center_y), 4, (0, 255, 255), -1)
    cv2.putText(image, "Home", (center_x + 5, center_y - 5),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    
    # Display robot coordinates
    coord_text = f"({x:.0f},{y:.0f})cm"
    cv2.putText(image, coord_text, (px + 5, py - 5),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    return image

def draw_shapes_and_colors(image):
    """Detect and display shapes and colors"""
    height, width = image.shape[:2]
    
    # Process image for shape and color detection
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    shapes_detected = []                # List to store detected shapes
    
    # Process each color in COLOR_RANGES
    for color_name, ranges in COLOR_RANGES.items():
        color_mask = np.zeros_like(hsv[:, :, 0])
        # Combine all ranges for this color (some colors have multiple ranges)
        for lower, upper in ranges:
            color_mask |= cv2.inRange(hsv, lower, upper)
        
        processed_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel, iterations=2)
        contours, _ = cv2.findContours(processed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 400:
                continue
            
            # Identify shape
            shape_name, approx = identify_shape(cnt)
            if shape_name == "Unknown":
                continue
            
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # Draw contour on main image
            cv2.drawContours(image, [approx], -1, (255, 255, 0), 2)
            
            # Display label on shape
            color_val = text_colors.get(color_name, (255, 255, 255))
            cv2.putText(image, f"{color_name} {shape_name}",
                       (cX - 60, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                       color_val, 2)
            
            # Store information for panel display
            shapes_detected.append(f"{color_name} {shape_name}")
    
    return image, shapes_detected

# Start serial reading thread
serial_thread = threading.Thread(target=serial_reader, daemon=True)
serial_thread.start()

# =========================
# === 10. Home Return Control ===
# =========================
def initiate_return_home():
    """Start returning home with 'h' key press - point by point with GOTO"""
    global retrace_in_progress, goto_done
    # Check if return conditions are met
    if not (at_aruco_point and mission_complete):
        print(" Return conditions not met: haven't reached Tag 5 or mission not complete.")
        return False

    print(" Starting return home process...")
    retrace_in_progress = True

    # Step 1: 180 degree turn to start return
    print(" Sending 180 degree turn command...")
    ser.write(b'TURN180\n')
    time.sleep(1.5)  # Wait for turn

    # Step 2: Get reverse path
    reverse_path = path_history.get_reverse_path()
    if len(reverse_path) < 2:
        print(" Not enough path for return!")
        retrace_in_progress = False
        return False

    print(f" Starting return with {len(reverse_path)-1} points...")

    # Step 3: Send reverse points (from second last to first, excluding last Tag5 and first home)
    for i in range(len(reverse_path)-2, -1, -1):
        point = reverse_path[i]
        print(f" Going to point {i}: X={point['x']:.1f}, Y={point['y']:.1f}, theta={point['theta']:.1f}")
        # Send with theta (new format)
        cmd = f"GOTO:{point['x']:.1f},{point['y']:.1f},{point['theta']:.0f}\n"
        ser.write(cmd.encode())

        # Wait for confirmation from Arduino (checked in serial_reader, wait loop here)
        wait_start = time.time()
        goto_done = False  # Reset before waiting
        while not goto_done:
            time.sleep(0.1)
            if time.time() - wait_start > 10:  # timeout 10 seconds
                print(" Timeout waiting to reach point!")
                break

    print(" Robot successfully returned home! ")
    retrace_in_progress = False
    return True

# =========================
# === 11. Main Loop ===
# =========================

frame_count = 0                      # Frame counter for FPS calculation
start_time = time.time()             # Start time for FPS calculation
last_aruco_check_time = time.time()  # Last time ArUco was checked
# New variable to prevent repeated ARUCO5_DETECTED messages
aruco5_notified = False
# New variable to check if moving towards Tag 5
moving_to_tag5 = False

try:
    while True:
        if odom_data['y'] > 160 :                         
            at_aruco_point = True
            mission_complete = True
            moving_to_tag5 = False
            ser.write(b's 0 0\n')    # stop
            ser.write(b'e 127 86\n') # aruco 5 detected to arduino

        # Capture image
        image = cam.capture_array("main")
        image = cv2.rotate(image, cv2.ROTATE_180)   # Rotate 180 degrees if camera is inverted
        image = cv2.medianBlur(image, 5)            # Apply median blur to reduce noise

        height, width = image.shape[:2]
        
        # ======================
        # ARUCO DETECTION
        # ======================

        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_parameters)
        
        detected_ids = []     # List to store detected ArUco IDs
        tag5_center_x = None  # For moving towards Tag 5
        
        if ids is not None:
            # Draw ArUco markers
            aruco.drawDetectedMarkers(image, corners, ids)
            
            # Check if ArUco Tag 5 seen
            current_time = time.time()
            if current_time - last_aruco_check_time > 0.5:  # Check every 0.5 seconds
                for i, marker_id in enumerate(ids):
                    area = abs(corners[0][0][1][0] - corners[0][0][0][0]) * abs(corners[0][0][2][1] - corners[0][0][1][1])
                    
                    if marker_id[0] == 5 and not mission_complete and area > 20000: # 23000 without search , 21000 with search 
                        print(f" ArUco Tag 5 detected! (ID: {marker_id[0]})")
                        print(f"   Current IR distance: {sensor_data['ir']} cm")
                      
                        c = True
                        # aruco_tag_5_detected

                        # Inform Arduino only once
                        if not aruco5_notified:
                            ser.write(b'ARUCO5_DETECTED\n')  # Inform Arduino
                            aruco5_notified = True
                            print("   Notification sent to Arduino")
                        
                        # Calculate Tag center for steering
                        marker_corners = corners[i][0]
                        tag5_center_x = int(np.mean(marker_corners[:, 0]))
                        
                        # Start moving towards Tag 5
                        moving_to_tag5 = True
                        
                        # If distance less than 15cm, give warning
                        if sensor_data['ir'] < 15:
                            print(f"   WARNING: Distance less than 15cm!")
                        
                        last_aruco_check_time = current_time
            
            # Store detected IDs
            for i, marker_id in enumerate(ids):
                detected_ids.append(marker_id[0])
                
                # Display ID on image
                marker_corners = corners[i][0]
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))
                
                # Different color for Tag 5
                if marker_id[0] == 5:
                    color = (0, 0, 255)  # Red for Tag 5
                    cv2.putText(image, f"TARGET ID: {marker_id[0]}", 
                               (center_x - 40, center_y - 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 3)
                else:
                    color = (0, 255, 255)  # Yellow for other Tags
                    cv2.putText(image, f"ID: {marker_id[0]}", 
                               (center_x - 20, center_y - 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # If scanning, store IDs
            if waiting_for_scan and detected_ids:
                direction_data[current_direction].extend(detected_ids)
                print(f" Stored {len(detected_ids)} IDs in direction {current_direction}: {detected_ids}")
                waiting_for_scan = False
            elif waiting_for_scan and not detected_ids:
                print(f" No IDs found in direction {current_direction}")
                waiting_for_scan = False
        
        # ======================
        # Move towards Tag 5 if seen and haven't arrived
        # ======================
        if moving_to_tag5 and not mission_complete:
            if tag5_center_x is not None:
                # Calculate deviation from image center
                image_center_x = width // 2
                deviation = tag5_center_x - image_center_x

                # Calculate normalized deviation (-1 to 1)
                normalized_deviation = deviation / (width // 2)
                
                # Set steering angle: 85 default (straight), adjust based on deviation
                # 70-100 range for left/right steering
                angle = 85 + normalized_deviation * 15  # Reduce sensitivity for smoother movement
                angle = max(70, min(100, int(angle)))   # Limit angle range
                
                # Only move if IR distance more than 10cm
                if sensor_data['ir'] > 10:
                    cmd = f"f 127 {angle}\n"
                    ser.write(cmd.encode())
                    print(f" Moving towards Tag 5 with angle={angle} (deviation={deviation}, IR={sensor_data['ir']}cm)")
                else:
                    # If IR < 10cm, stop and complete mission
                    if not at_aruco_point:
                        print(" Reached Tag 5 (IR < 10cm) - Stop and wait for return")
                        at_aruco_point = True
                        mission_complete = True
                        moving_to_tag5 = False
                        ser.write(b's 0 0\n')    # stop
                        ser.write(b'e 127 86\n') # aruco 5 detected to arduino


        
        # ======================
        # DETECT SHAPES AND COLORS
        # ======================
        image, shapes_detected = draw_shapes_and_colors(image)
        
        # ======================
        # CREATE INFO PANEL
        # ======================
        # Create info panel on left
        info_panel = np.zeros((300, 400, 3), dtype=np.uint8)
        
        # Title
        cv2.putText(info_panel, "=== ROBOT INFORMATION ===", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Odometry information
        cv2.putText(info_panel, f"Position X: {odom_data['x']:.1f} cm", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.putText(info_panel, f"Position Y: {odom_data['y']:.1f} cm", 
                   (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.putText(info_panel, f"Heading theta: {odom_data['theta']:.1f}degree", 
                   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Sensor information
        cv2.putText(info_panel, f"IR Front: {sensor_data['ir']} cm", 
                   (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                   (0, 255, 0) if sensor_data['ir'] > 14 else 
                   (0, 255, 255) if sensor_data['ir'] > 10 else 
                   (0, 0, 255), 1)
        
        cv2.putText(info_panel, f"US Right: {sensor_data['us1']} cm", 
                   (10, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                   (0, 255, 0) if sensor_data['us1'] > 12 else (0, 255, 255), 1)
        
        cv2.putText(info_panel, f"US Left: {sensor_data['us2']} cm", 
                   (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                   (0, 255, 0) if sensor_data['us2'] > 12 else (0, 255, 255), 1)
        
        # ArUco information
        cv2.putText(info_panel, f"Direction: {current_direction}", 
                   (10, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        # Display scan status
        if at_aruco_point:
            status = "AT TAG 5 - PRESS 'h' TO RETURN HOME"
            color = (0, 0, 255)  # Red
        elif moving_to_tag5:
            status = "MOVING TO TAG 5..."
            color = (255, 0, 0)  # Blue
        elif retrace_in_progress:
            status = "RETURNING HOME..."
            color = (255, 0, 0)  # Blue
        elif waiting_for_scan:
            status = "SCANNING..."
            color = (0, 255, 255)  # Yellow
        elif scan_complete:
            status = "SCAN COMPLETE"
            color = (0, 255, 0)    # Green
        else:
            status = "EXPLORING"
            color = (255, 255, 255)  # White
        
        cv2.putText(info_panel, f"Status: {status}", 
                   (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
        
        # Path history information
        path_count = path_history.get_point_count()
        cv2.putText(info_panel, f"Path Points: {path_count}", 
                   (10, 265), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                   (255, 255, 0) if path_count > 0 else (200, 200, 200), 1)
        
        # Display mission status
        if mission_complete:
            mission_text = "MISSION COMPLETE"
            mission_color = (0, 255, 0)  # Green
        elif moving_to_tag5:
            mission_text = "MOVING TO TARGET"
            mission_color = (255, 0, 0)  # Blue
        else:
            mission_text = "MISSION IN PROGRESS"
            mission_color = (255, 255, 0)  # Yellow
        cv2.putText(info_panel, f"Mission: {mission_text}", 
                   (10, 315), cv2.FONT_HERSHEY_SIMPLEX, 0.6, mission_color, 1)
        
        # Display detected IDs
        if detected_ids:
            unique_ids = list(set(detected_ids))
            if 5 in unique_ids:
                ids_text = f"ArUco IDs: {sorted(unique_ids)} [TARGET FOUND!]"
                cv2.putText(info_panel, ids_text, 
                           (10, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            elif len(unique_ids) <= 3:
                ids_text = f"ArUco IDs: {sorted(unique_ids)}"
                cv2.putText(info_panel, ids_text, 
                           (10, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            else:
                cv2.putText(info_panel, f"ArUco IDs: {len(unique_ids)} tags", 
                           (10, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # ======================
        # CREATE MAP PANEL
        # ======================
        # Create map panel on right
        map_panel = np.zeros((300, 300, 3), dtype=np.uint8)
        map_panel = draw_robot_position(map_panel, 
                                       odom_data['x'], 
                                       odom_data['y'], 
                                       odom_data['theta'])
        
        # ======================
        # COMBINE ALL PANELS
        # ======================
        # Create final image
        final_image = np.zeros((600, 800, 3), dtype=np.uint8)
        
        # Place main image on top
        final_image[0:height, 0:width] = image
        
        # Place info panel on bottom left
        final_image[300:600, 0:400] = info_panel
        
        # Place map panel on bottom right
        final_image[300:600, 400:700] = map_panel
        
        # Add title for map panel
        cv2.putText(final_image, "ROBOT POSITION MAP ", 
                   (410, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # ======================
        # FPS DISPLAY
        # ======================
        frame_count += 1
        elapsed_time = time.time() - start_time
        if elapsed_time > 0:
            average_fps = frame_count / elapsed_time
            fps_text = f"FPS: {average_fps:.1f}"
            cv2.putText(final_image, fps_text, 
                       (700, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
        # Display stored data in direction_data
        y_pos = 120
        for direction, ids_list in direction_data.items():
            if ids_list:
                unique_ids = sorted(set(ids_list))
                info = f"{direction}: {unique_ids}"
                cv2.putText(final_image, info, 
                           (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                y_pos += 25

        # ======================
        # DISPLAY WINDOW
        # ======================
        cv2.imshow("Robot Camera - ArUco, Shapes, Colors & Odometry", final_image)
        
        # ======================
        # KEYBOARD CONTROLS
        # ======================
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("Quitting...")
            ser.write(b's 0 0\n')
            break
        elif key == ord('s'):
            message = "s 0 0\n"
            ser.write(message.encode())
            print("Sent: Stop")
        elif key in [ord('f'), ord('b'), ord('l'), ord('r')]:
            cmd = chr(key)
            message = f"{cmd} 127 84\n"
            ser.write(message.encode())
            print(f"Sent: {message.strip()}")
        elif key == ord('h'):  # Return home
            ser.write(b'h  127 85\n')
            print("\n 'h' key pressed - starting return home...")
        elif key == ord('c'):  # Clear path history
            ser.write(b'CLEAR_PATH\n')
            path_history.clear()
            aruco5_notified = False  # Reset notification flag
            moving_to_tag5 = False   # Reset moving to Tag 5 state
            print("Sent: CLEAR_PATH")
        elif key == ord('p'):  # Print path history
            ser.write(b'PRINT_PATH\n')
            path_history.print_history()
            print("Sent: PRINT_PATH")
        elif key == ord('e'):  # Emergency stop and save position
            ser.write(b'e 0 0\n')
            path_history.add_point(odom_data['x'], odom_data['y'], odom_data['theta'], scan_point=True, sensor_data=sensor_data.copy())
            print(" Emergency stop and position saved")
        
        # If scan complete, wait for next scan start
        if scan_complete:
            time.sleep(1)
            scan_complete = False
            current_direction = "n0"
            
finally:
    # Cleanup when program exits
    print("Closing serial port...")
    ser.write(b's 0 0\n')   # Send stop command
    ser.close()
    cam.stop()
    cv2.destroyAllWindows() # Close all OpenCV windows