import RPi.GPIO as GPIO
import time

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# GPIO pin assignments for motors and sensors
IN1 = 17
IN2 = 18
ENA = 27
IN3 = 22
IN4 = 23
ENB = 24
SENSOR1 = 5
SENSOR2 = 6
SENSOR3 = 13
SENSOR4 = 20
SENSOR5 = 19

# Define the GPIO pins for TRIG and ECHO
TRIG = 25
ECHO = 26

# Set up the TRIG as output and ECHO as input
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Setup motor pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Setup sensor pins
GPIO.setup(SENSOR1, GPIO.IN)
GPIO.setup(SENSOR2, GPIO.IN)
GPIO.setup(SENSOR3, GPIO.IN)
GPIO.setup(SENSOR4, GPIO.IN)
GPIO.setup(SENSOR5, GPIO.IN)

# Initialize PWM on ENA and ENB pins
pwmA = GPIO.PWM(ENA, 1000)  # 1000 Hz frequency
pwmB = GPIO.PWM(ENB, 1000)  # 1000 Hz frequency
pwmA.start(0)  # Start PWM with 0% duty cycle
pwmB.start(0)  # Start PWM with 0% duty cycle

def set_speed(motorA_speed, motorB_speed):
    pwmA.ChangeDutyCycle(motorA_speed)
    pwmB.ChangeDutyCycle(motorB_speed)

def move_forward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def sensor_readings():
    # Read sensor values (1 means white line detected, 0 means black surface)
    s1 = GPIO.input(SENSOR1)
    s2 = GPIO.input(SENSOR2)
    s3 = GPIO.input(SENSOR3)
    s4 = GPIO.input(SENSOR4)
    s5 = GPIO.input(SENSOR5)
    return [s1, s2, s3, s4, s5]

# PID Controller parameters
KP = 12  # Proportional gain
KI = 0.1  # Integral gain
KD = 10  # Derivative gain
BASE_SPEED = 50  # Base speed for motors (0 to 100)

previous_error = 0
integral = 0
previous_time = time.time()

def calculate_error(sensor_values):
    # Assign weights to sensors: [leftmost, left, center, right, rightmost]
    WEIGHTS = [-2, -1, 0, 1, 2]
    error = sum(weight * value for weight, value in zip(WEIGHTS, sensor_values))
    return error

def detect_junction(sensor_values):
    # Define patterns for different junction types
    CROSS_JUNCTION = [1, 1, 1, 1, 1]
    T_JUNCTION = [1, 1, 1, 0, 0]
    LEFT_L_JUNCTION = [1, 1, 0, 0, 0]
    RIGHT_L_JUNCTION = [0, 0, 0, 1, 1]

    if sensor_values == CROSS_JUNCTION:
        return "Cross"
    elif sensor_values[0:2] == [1, 1, 1]:
        return "T"
    elif sensor_values[0] == 1 and sensor_values[1] == 1:
        return "Left L"
    elif sensor_values[4] == 1 and sensor_values[3] == 1:
        return "Right L"
    return "None"

def line_following():
    global previous_error, integral, previous_time

    while True:
        sensor_values = sensor_readings()
        error = calculate_error(sensor_values)

        junction = detect_junction(sensor_values)
        if junction != "None":
            print(f"{junction} junction detected")
            stop()
            time.sleep(1.5)
            # Handle the junction appropriately
            if junction == "Cross":
                move_forward()
                set_speed(BASE_SPEED, BASE_SPEED)
                time.sleep(1.5)
            elif junction == "T":
                # Example action: turn right at T junction
                print("Turning right at T junction")
                GPIO.output(IN1, GPIO.HIGH)
                GPIO.output(IN2, GPIO.LOW)
                GPIO.output(IN3, GPIO.HIGH)
                GPIO.output(IN4, GPIO.LOW)
                set_speed(50, 50)
                time.sleep(0.2)  # Initial delay
                stop()
            elif junction == "Left L":
                move_forward()
                set_speed(BASE_SPEED, BASE_SPEED)
                time.sleep(1.5)
            elif junction == "Right L":
                move_forward()
                set_speed(BASE_SPEED, BASE_SPEED)
                time.sleep(1.5)

        current_time = time.time()
        dt = current_time - previous_time

        proportional = error
        integral += error * dt
        derivative = (error - previous_error) / dt if dt > 0 else 0

        correction = (KP * proportional) + (KI * integral) + (KD * derivative)

        left_speed = BASE_SPEED - correction
        right_speed = BASE_SPEED + correction

        # Clamp the speeds between 0 and 100
        left_speed = max(min(left_speed, 100), 0)
        right_speed = max(min(right_speed, 100), 0)

        move_forward()
        set_speed(left_speed, right_speed)

        previous_error = error
        previous_time = current_time

        time.sleep(0.01)

try:
    line_following()
except KeyboardInterrupt:
    pass

# Cleanup
stop()
pwmA.stop()
pwmB.stop()
GPIO.cleanup()
