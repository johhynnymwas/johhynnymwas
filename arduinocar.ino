// Define the pins for the IR sensors
const int IR1 = A0;
const int IR2 = A1;
const int IR3 = A2;
const int IR4 = A3;
const int IR5 = A4;

// Threshold value for considering a sensor reading as detecting the white line
const int IR_THRESHOLD = 500;

// Define motor control pins
const int motorLeftENA = 6;    // ENA pin for left motor
const int motorLeftIN1 = 7;     // IN1 pin for left motor
const int motorLeftIN2 = 8;     // IN2 pin for left motor
const int motorRightENA = 9;    // ENA pin for right motor
const int motorRightIN3 = 10;   // IN3 pin for right motor
const int motorRightIN4 = 11;   // IN4 pin for right motor

// Motor speeds
const int BASE_SPEED = 150;     // Base speed for motors
const int TURN_SPEED = 200;     // Speed for turning

void setup() {
  // Initialize IR sensor pins as inputs
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  // Initialize motor control pins as outputs
  pinMode(motorLeftENA, OUTPUT);
  pinMode(motorLeftIN1, OUTPUT);
  pinMode(motorLeftIN2, OUTPUT);
  pinMode(motorRightENA, OUTPUT);
  pinMode(motorRightIN3, OUTPUT);
  pinMode(motorRightIN4, OUTPUT);

  // Begin Serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read sensor values
  int ir1_val = analogRead(IR1);
  int ir2_val = analogRead(IR2);
  int ir3_val = analogRead(IR3);
  int ir4_val = analogRead(IR4);
  int ir5_val = analogRead(IR5);

  // Print sensor values
  Serial.print("IR1: ");
  Serial.print(ir1_val);
  Serial.print(" IR2: ");
  Serial.print(ir2_val);
  Serial.print(" IR3: ");
  Serial.print(ir3_val);
  Serial.print(" IR4: ");
  Serial.print(ir4_val);
  Serial.print(" IR5: ");
  Serial.println(ir5_val);

  // Thresholding
  bool ir1_detected = ir1_val < IR_THRESHOLD;
  bool ir2_detected = ir2_val < IR_THRESHOLD;
  bool ir3_detected = ir3_val < IR_THRESHOLD;
  bool ir4_detected = ir4_val < IR_THRESHOLD;
  bool ir5_detected = ir5_val < IR_THRESHOLD;

  // Print base speed
  Serial.print("Base Speed: ");
  Serial.println(BASE_SPEED);

  // Decision Making
  if (ir1_detected && ir2_detected && ir3_detected && ir4_detected && ir5_detected) {
    moveForward();
  } else if (ir1_detected || ir2_detected) {
    turnRight();
  } else if (ir4_detected || ir5_detected) {
    turnLeft();
  } else {
    stop();
  }
}

// Function to move forward
void moveForward() {
  digitalWrite(motorLeftIN1, HIGH);
  digitalWrite(motorLeftIN2, LOW);
  analogWrite(motorLeftENA, BASE_SPEED);

  digitalWrite(motorRightIN3, HIGH);
  digitalWrite(motorRightIN4, LOW);
  analogWrite(motorRightENA, BASE_SPEED);

  // Print center of the line
  Serial.println("Center of the line");
}

// Function to turn right
void turnRight() {
  digitalWrite(motorLeftIN1, HIGH);
  digitalWrite(motorLeftIN2, LOW);
  analogWrite(motorLeftENA, TURN_SPEED);

  digitalWrite(motorRightIN3, LOW);
  digitalWrite(motorRightIN4, HIGH);
  analogWrite(motorRightENA, TURN_SPEED);

  // Print center of the line
  Serial.println("Turning Right");
}

// Function to turn left
void turnLeft() {
  digitalWrite(motorLeftIN1, LOW);
  digitalWrite(motorLeftIN2, HIGH);
  analogWrite(motorLeftENA, TURN_SPEED);

  digitalWrite(motorRightIN3, HIGH);
  digitalWrite(motorRightIN4, LOW);
  analogWrite(motorRightENA, TURN_SPEED);

  // Print center of the line
  Serial.println("Turning Left");
}

// Function to stop
void stop() {
  digitalWrite(motorLeftENA, LOW);
  digitalWrite(motorRightENA, LOW);

  // Print center of the line
  Serial.println("Stopped");
}
