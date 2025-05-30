import cv2
import time
import busio
import board
import smbus2
import numpy as np
from time import sleep
import RPi.GPIO as GPIO
import adafruit_vl53l0x
from picamera2 import Picamera2

# I2C and Multiplexer Configuration
TCA9548A_ADDRESS = 0x70
TCA_CHANNELS = [0, 1, 2]   # Channels for sensors (object detection and bins)

# GPIO Pin Definitions
DC_MOTOR_IN1 = 17
DC_MOTOR_IN2 = 25
DC_MOTOR_ENA = 12
RESET_DELAY = 0.5
SQUARE_SERVO = 18
TRIANGLE_SERVO = 19

# Threshold distance for detecting presence
DETECTION_THRESHOLD_STOP = 100  # Threshold for stopping conveyor (mm)
DETECTION_THRESHOLD_BINS = 100  # Threshold for detecting collected objects (mm)

# Camera Initialization
try:
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()
except Exception as e:
    print(f"Error initializing Picamera2: {e}")
    exit(1)

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# GPIO Initialization
GPIO.setmode(GPIO.BCM)
GPIO.setup(DC_MOTOR_IN1, GPIO.OUT)
GPIO.setup(DC_MOTOR_IN2, GPIO.OUT)
GPIO.setup(DC_MOTOR_ENA, GPIO.OUT)
GPIO.setup(SQUARE_SERVO, GPIO.OUT)
GPIO.setup(TRIANGLE_SERVO, GPIO.OUT)

# Motor and Servo Initialization
dc_motor = GPIO.PWM(DC_MOTOR_ENA, 1000)
square_servo = GPIO.PWM(SQUARE_SERVO, 50)
triangle_servo = GPIO.PWM(TRIANGLE_SERVO, 50)
dc_motor.start(0)
square_servo.start(0)
triangle_servo.start(0)

# Motor and Servo Control Functions
def motor_forward(speed=35):
    GPIO.output(DC_MOTOR_IN1, GPIO.HIGH)
    GPIO.output(DC_MOTOR_IN2, GPIO.LOW)
    dc_motor.ChangeDutyCycle(speed)

def motor_backward(speed=35):
    GPIO.output(DC_MOTOR_IN1, GPIO.LOW)
    GPIO.output(DC_MOTOR_IN2, GPIO.HIGH)
    dc_motor.ChangeDutyCycle(speed)

def motor_stop(speed=0):
    GPIO.output(DC_MOTOR_IN1, GPIO.LOW)
    GPIO.output(DC_MOTOR_IN2, GPIO.LOW)
    dc_motor.ChangeDutyCyc">"

def move_servo(servo, angle):
    duty_cycle = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(RESET_DELAY)
    servo.ChangeDutyCycle(0)

def reset_servo(servo):
    move_servo(servo, 0)

# Function to enable a specific channel on the TCA9548A
def select_channel(bus, address, channel):
    if channel < 0 or channel > 7:
        raise ValueError("Invalid channel: must be 0-7")
    bus.write_byte(address, 1 << channel)

# OpenCV Parameter Configuration
def empty(x):
    pass

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640, 240)
cv2.createTrackbar("Threshold1", "Parameters", 255, 255, empty)
cv2.createTrackbar("Threshold2", "Parameters", 255, 255, empty)
cv2.createTrackbar("Area", "Parameters", 8552, 20000, empty)

# Original getContour Function
def getContours(imgDil, imgContour):
    global shape
    contours, _ = cv2.findContours(imgDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        areaMin = cv2.getTrackbarPos("Area", "Parameters")
        area = cv2.contourArea(cnt)
        if area > areaMin:
            print(f"Area: {area}")
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 3)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            print(f"Points: {len(approx)}")
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.putText(imgContour, f"Points: {len(approx)}", (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255), 1)
            cv2.putText(imgContour, f"Area: {int(area)}", (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255), 1)
            aspect_ratio = float(w) / h if h > 0 else 0
            circularity = (4 * np.pi * area) / (peri ** 2) if peri > 0 else 0
            edges = len(approx)
            
            # Shape definition
            if edges == 3:
                shape = "Triangle"
            elif edges == 4 and 0.95 <= aspect_ratio <= 1.05:
                shape = "Square"
            elif circularity > 0.85 and 0.95 <= aspect_ratio <= 1.05:
                shape = "Circle"
            else:
                shape = "Unknown"

            print(f"Detected Product: {shape}")
            cv2.putText(imgContour, shape + " Product", (x + w + 20, y + 70), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 1)

        # Sorting Logic
        previous_shape = None
        if shape != previous_shape:
                previous_shape = shape
                if shape == "Square":
                    move_servo(square_servo, 80)
                    time.sleep(1)
                    motor_forward(35) 
                elif shape == "Triangle":
                    move_servo(triangle_servo, 70)
                    time.sleep(1)
                    motor_forward(35) 
                elif shape == "Circle":  # No sorting needed for circle product
                    time.sleep(1)
                    motor_forward(30)   
                elif shape == "Unknown":
                    motor_backward(40)
                    time.sleep(1)
                    motor_forward(35)

# Initialize multiplexer (using smbus2)
bus = smbus2.SMBus(1)

# Initialize sensors
sensors = []
for channel in TCA_CHANNELS:
    select_channel(bus, TCA9548A_ADDRESS, channel)
    sensor = VL53L0X(i2c)
    sensors.append(sensor)

# Test the sensors and control the motor
try:
    while True:
        
        # Select Channel 1 (index 0 in the list)
        select_channel(bus, TCA9548A_ADDRESS, TCA_CHANNELS[0])
        distance = sensors[0].range  # Read distance from Channel 1
        print(f"Sensor 1 (Channel 1): {distance} mm")
        
        if distance < DETECTION_THRESHOLD_STOP:
            print("Product on the conveyor, stopping motor...")
            motor_stop()
        else:
            print("No product on the conveyor, running motor...")
            motor_forward(35)  # Adjust speed as needed (0-100)
        
        sleep(0.1)

        # Select Channel 2 (index 1 in the list)
        select_channel(bus, TCA9548A_ADDRESS, TCA_CHANNELS[1])
        distance = sensors[1].range  # Read distance from Channel 2
        print(f"Sensor 2 (Channel 2): {distance} mm")
        
        if distance < DETECTION_THRESHOLD_BINS:
            print("Product collected in bin 1 , resetting square servo motor...")
            reset_servo(square_servo)
        else:
            print("No product collected in bin 1.")
        
        sleep(0.1)

        # Select Channel 3 (index 2 in the list)
        select_channel(bus, TCA9548A_ADDRESS, TCA_CHANNELS[2])
        distance = sensors[2].range  # Read distance from Channel 3
        print(f"Sensor 3 (Channel 3): {distance} mm")
        
        if distance < DETECTION_THRESHOLD_BINS:
            print("Product collected on bin 2, reseting triangle servo motor...")
            reset_servo(triangle_servo)
        else:
            print("No product collected in bin 2.")
        
        sleep(0.1)

        # Capture a frame from the camera
        frame = picam2.capture_array()
        imgContour = frame.copy()

         # Preprocess the image
        imgBlur = cv2.GaussianBlur(frame, (7, 7), 1)
        imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)

        # Get thresholds from trackbars
        t1 = cv2.getTrackbarPos("Threshold1", "Parameters")
        t2 = cv2.getTrackbarPos("Threshold2", "Parameters")
        imgCanny = cv2.Canny(imgGray, t1, t2)

        # Dilate the edges
        kernel = np.ones((5, 5))
        imgDil = cv2.dilate(imgCanny, kernel, iterations=1)

        # Detect and draw contours
        getContours(imgDil, imgContour=imgContour)

        # Ensure all images have the same dimensions and color format
        frame_resized = cv2.resize(frame, (640, 480))
        imgDil_resized = cv2.resize(imgDil, (640, 480))
        imgContour_resized = cv2.resize(imgContour, (640, 480))

        # Convert grayscale images to BGR for stacking
        imgDil_resized = cv2.cvtColor(imgDil_resized, cv2.COLOR_GRAY2BGR)

        # Stack images for visualization
        imgStack = np.hstack([frame_resized, imgDil_resized, imgContour_resized])
        getContours(imgDil, imgContour)

        # Display images for debugging
        cv2.imshow("Output", imgStack)

        # Exit condition
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
except KeyboardInterrupt:
    print("Exiting...")

finally:
    
    # Stop the camera and release resources
    print("Cleaning up...")
    dc_motor.stop()
    square_servo.stop()
    triangle_servo.stop()
    picam2.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    bus.close()