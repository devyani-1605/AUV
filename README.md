
# Autonomous Underwater Vehicle (AUV)

This project demonstrates the design and implementation of an Autonomous Underwater Vehicle (AUV) using a Raspberry Pi, an Arduino, and various sensors. The system combines hardware and software components for precise underwater navigation, object detection, and motor control.

## Table of Contents

 1 Features
 
 2 Hardware Requirements

 3 Software Requirements

 4 Setup and Usage

 5 Usage Scenarios
 
## Features

- PID Control: Precise motor movement using a PID control loop.
- IMU-Based Orientation Tracking: Real-time orientation measurement using the BNO055 IMU sensor.
- Color Detection: Detection of orange and red objects using HSV color masks.
- Line Following and Object Calibration: Navigation logic for following lines and detecting calibrated objects.
- Multithreading: Efficient handling of multiple tasks (e.g., PID control, object detection) simultaneously.
- Serial Communication: Communication with an Arduino for motor control.
- Modular Design: Flexibility to add more features or customize hardware setups.



## Hardware Requirements

- Raspberry Pi (with GPIO support)
- IMU Sensor (BNO055)
- Bi-Directional Motors
- Camera Module (compatible with OpenCV)
- Arduino (or equivalent microcontroller)
- Serial-to-USB Cable

## Software Requirements

1.Python 3.8+

2.Required Python Libraries:
 - OpenCV for image processing
 - NumPy for numerical computations
 - gpiozero for GPIO control
 - Adafruit-BNO055 for interfacing with the IMU sensor
 - pyserial for serial communication
 - tf-transformations for quaternion to Euler angle conversions
- board and busio libraries for I2C communication (installed with Adafruit-BNO055)

## Install Dependencies

```bash
  pip install opencv-python numpy gpiozero adafruit-circuitpython-bno055 pyserial tf-transformations
```



## Setup and Usage

## Hardware Setup
1.IMU Sensor: Connect the BNO055 IMU sensor to the Raspberry Pi via I2C pins.

2.Motor Connection: Connect the motors to the GPIO pins and ensure proper wiring. Use GPIO pin 17 for motor control (modifiable in the code).

3.Camera Setup: Attach the camera module and test its functionality using OpenCV.

4.Arduino: Connect the Arduino to the Raspberry Pi via a Serial-to-USB cable.

## Run Locally

Clone the project

```bash
  git clone https://github.com/devyani-1605/AUV.git
```

Go to the project directory

```bash
  cd my-project
```
Run the code 
```bash
  python3 main.py


```
## Calibrate Sensors

1.IMU Calibration: Ensure the BNO055 IMU is properly calibrated.

2.Color Detection: Use the color detection functions to verify the HSV ranges for the target environment.

## Notes

- PID Calibration: Adjust kp, ki, and kd values in the code to optimize motor control based on hardware setup.



