# Conveyor Shape Detection

This is a Python project designed to run on a Raspberry Pi, connected to a conveyor system equipped with a camera and laser sensors. The system automatically detects geometric shapes (such as circles, squares, and triangles) on moving objects and sorts them accordingly using servo motors.

## Features

- Real-time object detection using OpenCV
- Shape classification (circle, square, triangle)
- Raspberry Pi GPIO integration for controlling:
  - Conveyor belt (DC motor)
  - Sorting mechanisms (servo motors)
  - Laser sensors for object position tracking
- Camera-based vision system using Picamera2
- Efficient hardware-software coordination

## Hardware Requirements

- Raspberry Pi (with GPIO access)
- Picamera2 module
- 3x VL53L0X laser sensors
- 2x Servo motors
- 1x DC motor (for conveyor belt)
- External power supply (12V for motors)
- Breadboard, wires, resistors

## Software Requirements

- Python 3
- OpenCV (\`opencv-python\`)
- NumPy
- RPi.GPIO
- smbus2
- Adafruit VL53L0X
- Picamera2
- adafruit-blinka

\`\`\`bash
pip install opencv-python numpy smbus2 RPi.GPIO adafruit-blinka adafruit-circuitpython-vl53l0x
sudo apt install python3-picamera2
\`\`\`

## How It Works

1. An object breaks the first laser beam.
2. The conveyor pauses and a photo is taken.
3. OpenCV processes the image and detects the shape.
4. Based on the shape:
   - Square or triangle: servo activates and diverts the object.
   - Circle: continues straight into a collection bag.
5. System resets and waits for the next item.

## License

MIT License
