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

# Define servo pins
servos = {
    "base":4 ,
    "shoulder": 16,
    "elbow":21 ,
    "wrist": 7,
    "gripper": 8
}
# Set up servo pins
for pin in servos.values():
    GPIO.setup(pin, GPIO.OUT)

# Set up PWM channels with a frequency of 50Hz
pwm = {name: GPIO.PWM(pin, 50) for name, pin in servos.items()}
for p in pwm.values():
    p.start(0)
    
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

def set_servo_angle(servo_name, angle):
    duty_cycle = 2 + (angle / 18)
    pwm[servo_name].ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Wait for the servo to reach the position
    pwm[servo_name].ChangeDutyCycle(0)  # Turn off PWM signal to stop jitter

def pick_object():
    # Define the sequence of movements for picking up an object
    # Adjust these angles as per your robotic arm's configuration
    set_servo_angle("base", 90)      # Rotate base to the object
    set_servo_angle("shoulder", 45)  # Move shoulder down to reach the object
    set_servo_angle("elbow", 90)     # Adjust elbow to reach the object
    set_servo_angle("wrist", 0)      # Position wrist for picking
    set_servo_angle("gripper", 30)   # Open gripper

    time.sleep(1)  # Wait for a moment to ensure the arm is in position

    set_servo_angle("gripper", 90)   # Close gripper to grasp the object

    time.sleep(1)  # Ensure the object is gripped

    set_servo_angle("shoulder", 90)  # Lift the object
    set_servo_angle("elbow", 45)     # Adjust elbow to lift the object
    set_servo_angle("wrist", 90)     # Adjust wrist to lift the object

    time.sleep(1)  # Wait for the arm to stabilize
def place_object():
    set_servo_angle("base", 0)       # Rotate base to the target position
    set_servo_angle("shoulder", 45)  # Move shoulder down to place the object
    set_servo_angle("elbow", 90)     # Adjust elbow to place the object
    set_servo_angle("wrist", 0)      # Position wrist for placing
    set_servo_angle("gripper", 30)   # Open gripper to release the object

    time.sleep(1)  # Ensure the object is released

    # Return to the initial position
    set_servo_angle("shoulder", 90)
    set_servo_angle("elbow", 90)
    set_servo_angle("wrist", 90)
    set_servo_angle("gripper", 0)
    time.sleep(1)  # Wait for the arm to stabilize
def measure_distance():
    # Send a 10µs pulse to TRIG to start the measurement
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for the ECHO pin to go high and record the start time
    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    # Wait for the ECHO pin to go low and record the end time
    while GPIO.input(ECHO) == 1:
        end_time = time.time()

    # Calculate the duration of the pulse
    pulse_duration = end_time - start_time

    # Calculate the distance in cm (speed of sound is 34300 cm/s)
    distance = (pulse_duration * 34300) / 2

    return distance    
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

def turn_left():
    print("Turning left")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    set_speed(BASE_SPEED, BASE_SPEED)
    while GPIO.input(SENSOR4) == 1:
        time.sleep(0.01)
    print("Left turn completed")

def turn_right():
    print("Turning right")
    # Initial turn
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_speed(50, 50)
    time.sleep(0.2)  # Initial delay
    stop()
    z = 0

    while GPIO.input(SENSOR1) == 1 or GPIO.input(SENSOR3) == 1 or GPIO.input(SENSOR5) == 1:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        set_speed(80, 60)
        
        if z < 1:
            time.sleep(0.2)
        if z >= 100:
            z = 3
        z += 1

        if GPIO.input(SENSOR3) == 0 or GPIO.input(SENSOR5) == 0:
            stop()
            break
    print("Right turn completed")
def stop_right_wheel():
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwmB.ChangeDutyCycle(0)

def about_turn():
    print("Performing about-turn")
    move_backward()
    set_speed(BASE_SPEED, BASE_SPEED)
    time.sleep(1.0)  # Adjust time to complete the about-turn
    while GPIO.input(SENSOR3) == 1:
        time.sleep(0.01)
    move_forward()
    set_speed(BASE_SPEED, BASE_SPEED)
    time.sleep(1.0)  # Adjust time to complete the about-turn

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

def line_following():
    global previous_error, integral, previous_time
    
    cross_count = 0
    t_junction_count = 0
    l_junction_left_count = 0
    l_junction_right_count = 0

    while True:
        sensor_values = sensor_readings()
        error = calculate_error(sensor_values)
        # Print the error
        print(f"Sensor values: {sensor_values}, Error: {error}")
        # Measure distance and print the result
        dist = measure_distance()
        print(f"Distance: {dist:.1f} cm")
        time.sleep(1)  # Wait for 1 second before the next measurement

        # Detect different types of junctions
        if sensor_values == [1, 1, 1, 1, 1]:  # Cross junction
            print("Cross junction detected")
            stop()
            time.sleep(1.5)
            cross_count += 1
            move_forward()
            set_speed(BASE_SPEED, BASE_SPEED)
            print(f"Cross junction count: {cross_count}")
            time.sleep(1.5)

        elif sensor_values[1] == 1 and sensor_values[2] == 1 and sensor_values[3] == 1:  # T junction
            print("T junction detected")
            stop()
            time.sleep(1.5)
            t_junction_count += 1
            # Decide what to do at a T junction
            if t_junction_count == 1:
                print("First T junction, making a right turn")
                stop_right_wheel()  # Stop the right wheel
                time.sleep(1.5)  # Adjust this delay as needed to handle the turn
                print("Right wheel stopped at T junction")
                turn_right()

            
            move_forward()
            set_speed(BASE_SPEED, BASE_SPEED)
            print(f"T junction count: {t_junction_count}")
            time.sleep(1.5)

        elif sensor_values[1] == 1 and sensor_values[2] == 1:  # Left L junction
            print("Left L junction detected")
            stop()
            time.sleep(1.5)
            l_junction_left_count += 1
            move_forward()
            set_speed(BASE_SPEED, BASE_SPEED)
            print(f"Left L junction count: {l_junction_left_count}")
            time.sleep(1.5)

        elif sensor_values[4] == 1 and sensor_values[5] == 1:  # Right L junction
            print("Right L junction detected")
            stop()
            time.sleep(1.5)
            l_junction_right_count += 1
            move_forward()
            set_speed(BASE_SPEED, BASE_SPEED)
            print(f"Right L junction count: {l_junction_right_count}")
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

        print(f"Correction: {correction}, Left speed: {left_speed}, Right speed: {right_speed}")

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
