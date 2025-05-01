#include <Arduino.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "target_detection.h"
#include <ESP32Servo.h>

// Motor Driver Pins - L298N
#define ENA 10  // Enable1 L298 Pin enA
#define IN1 9   // Motor1 L298 Pin in1
#define IN2 8   // Motor1 L298 Pin in2
#define IN3 7   // Motor2 L298 Pin in3
#define IN4 6   // Motor2 L298 Pin in4
#define ENB 5   // Enable2 L298 Pin enB

// Sensor Pins
#define LEFT_IR_SENSOR 39  // IR sensor Left (A0 equivalent on ESP32-S3)
#define RIGHT_IR_SENSOR 40 // IR sensor Right (A1 equivalent on ESP32-S3)
#define ECHO_PIN 41        // Echo pin (A2 equivalent on ESP32-S3)
#define TRIGGER_PIN 42     // Trigger pin (A3 equivalent on ESP32-S3)
#define SCAN_SERVO_PIN 43  // Servo for scanning (A5 equivalent on ESP32-S3)

// Dart Turret Servo Pins
#define PAN_SERVO_PIN 12   // Horizontal movement
#define TILT_SERVO_PIN 13  // Vertical movement
#define TRIGGER_SERVO_PIN 14 // Trigger mechanism

// Constants
#define OBSTACLE_DISTANCE 15 // Minimum distance to obstacle (cm)
#define MOTOR_SPEED 200      // Default motor speed (0-255)
#define SCAN_INTERVAL 5000   // Time between target scans (ms)
#define DETECTION_THRESHOLD 0.6 // Confidence threshold for target detection

// Servo objects
Servo scanServo;
Servo panServo;
Servo tiltServo;
Servo triggerServo;

// Servo positions
#define PAN_MIN 0
#define PAN_MAX 180
#define TILT_MIN 45
#define TILT_MAX 135
#define TRIGGER_REST 90
#define TRIGGER_FIRE 0

// Global variables
unsigned long lastScanTime = 0;
bool targetDetected = false;
int currentPanPos = 90;  // Center position
int currentTiltPos = 90; // Center position
int distance_L, distance_F, distance_R;
bool scanningMode = true;
bool targetMode = false; // When true, robot stops to aim and fire

// Function prototypes
void setupCamera();
void setupMotors();
void setupServos();
void scanForTargets();
void fireAtTarget(int panAngle, int tiltAngle);
void moveServos(int panAngle, int tiltAngle);
bool captureAndAnalyzeImage(int* targetX, int* targetY, float* confidence);
void calculateFiringAngles(int targetX, int targetY, int* panAngle, int* tiltAngle);
long ultrasonicRead();
void compareDistance();
void checkSide();
void forward();
void backward();
void turnRight();
void turnLeft();
void stopMotors();
void lineFollowAndAvoidObstacles();

void setup() {
  Serial.begin(115200);
  Serial.println("Autonomous Dart Turret Robot Starting...");
  
  // Initialize motors
  setupMotors();
  
  // Initialize servos
  setupServos();
  
  // Initialize camera
  setupCamera();
  
  // Initial scan with ultrasonic sensor
  distance_F = ultrasonicRead();
  
  Serial.println("System ready!");
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // If in target mode, focus on aiming and firing
  if (targetMode) {
    scanForTargets();
    targetMode = false; // Return to line following after handling target
    return;
  }
  
  // Normal line following and obstacle avoidance
  lineFollowAndAvoidObstacles();
  
  // Periodically check for targets
  if (scanningMode && (currentTime - lastScanTime > SCAN_INTERVAL)) {
    Serial.println("Checking for targets...");
    
    // Stop the robot while scanning
    stopMotors();
    
    // Quick check for target
    int targetX, targetY;
    float confidence;
    
    if (captureAndAnalyzeImage(&targetX, &targetY, &confidence)) {
      Serial.println("Target detected! Switching to target mode");
      targetMode = true;
    }
    
    lastScanTime = currentTime;
  }
  
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 's': // Start scanning mode
        scanningMode = true;
        Serial.println("Scanning mode activated");
        break;
      case 'p': // Pause scanning mode
        scanningMode = false;
        Serial.println("Scanning mode paused");
        break;
      case 'f': // Force fire
        stopMotors();
        fireAtTarget(currentPanPos, currentTiltPos);
        break;
      case 'm': // Manual move servos
        if (Serial.available() >= 2) {
          int pan = Serial.parseInt();
          int tilt = Serial.parseInt();
          moveServos(pan, tilt);
        }
        break;
    }
  }
  
  delay(10); // Small delay to prevent CPU hogging
}

void setupMotors() {
  // Set all motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Set sensor pins
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  
  // Set initial motor speed
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
  
  // Initially stop motors
  stopMotors();
}

void setupServos() {
  // Initialize all servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  scanServo.setPeriodHertz(50);    // Standard 50hz servo
  panServo.setPeriodHertz(50);     // Standard 50hz servo
  tiltServo.setPeriodHertz(50);    // Standard 50hz servo
  triggerServo.setPeriodHertz(50); // Standard 50hz servo
  
  // Attach servos to pins
  scanServo.attach(SCAN_SERVO_PIN, 500, 2500);
  panServo.attach(PAN_SERVO_PIN, 500, 2500);
  tiltServo.attach(TILT_SERVO_PIN, 500, 2500);
  triggerServo.attach(TRIGGER_SERVO_PIN, 500, 2500);
  
  // Set to initial positions
  scanServo.write(70); // Center position for scanning servo
  panServo.write(90);  // Center position for pan
  tiltServo.write(90); // Center position for tilt
  triggerServo.write(TRIGGER_REST); // Rest position for trigger
  
  // Initial scan with servo
  for (int angle = 70; angle <= 140; angle += 5) {
    scanServo.write(angle);
    delay(50);
  }
  for (int angle = 140; angle >= 0; angle -= 5) {
    scanServo.write(angle);
    delay(50);
  }
  for (int angle = 0; angle <= 70; angle += 5) {
    scanServo.write(angle);
    delay(50);
  }
  
  delay(500); // Allow servos to reach position
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Initial settings for QVGA resolution
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12; // 0-63, lower is higher quality
  config.fb_count = 1;
  
  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  Serial.println("Camera initialized successfully");
}

void lineFollowAndAvoidObstacles() {
  // Read distance from ultrasonic sensor
  distance_F = ultrasonicRead();
  Serial.print("Distance Front: "); Serial.println(distance_F);
  
  // Read IR sensors
  bool rightSensor = digitalRead(RIGHT_IR_SENSOR);
  bool leftSensor = digitalRead(LEFT_IR_SENSOR);
  
  // Line following logic with obstacle avoidance
  if (!rightSensor && !leftSensor) { // Both sensors on white
    if (distance_F > OBSTACLE_DISTANCE) {
      forward();
    } else {
      checkSide(); // Obstacle detected, check sides
    }
  } else if (rightSensor && !leftSensor) { // Right sensor on black, left on white
    turnRight();
  } else if (!rightSensor && leftSensor) { // Right sensor on white, left on black
    turnLeft();
  } else { // Both sensors on black (might be a junction or end of line)
    // For now, just continue forward if no obstacle
    if (distance_F > OBSTACLE_DISTANCE) {
      forward();
    } else {
      checkSide();
    }
  }
}

void scanForTargets() {
  Serial.println("Scanning for targets...");
  
  // Variables to store target information
  int targetX, targetY;
  float confidence;
  
  // Scan pattern - horizontal sweep with turret
  for (int pan = PAN_MIN; pan <= PAN_MAX; pan += 20) {
    for (int tilt = TILT_MIN; tilt <= TILT_MAX; tilt += 20) {
      // Move to position
      moveServos(pan, tilt);
      delay(300); // Allow camera to stabilize
      
      // Capture and analyze image
      if (captureAndAnalyzeImage(&targetX, &targetY, &confidence)) {
        Serial.println("Target detected!");
        
        // Calculate firing angles
        int panAngle, tiltAngle;
        calculateFiringAngles(targetX, targetY, &panAngle, &tiltAngle);
        
        // Aim and fire
        fireAtTarget(panAngle, tiltAngle);
        
        // Return to scanning
        return;
      }
    }
  }
  
  Serial.println("No targets found during scan");
}

bool captureAndAnalyzeImage(int* targetX, int* targetY, float* confidence) {
  // Capture frame
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
  
  // Process image to detect target
  bool detected = detectTarget(fb->buf, fb->len, targetX, targetY, confidence);
  
  // Return the frame buffer back to be reused
  esp_camera_fb_return(fb);
  
  // Check if target detected with sufficient confidence
  if (detected && *confidence > DETECTION_THRESHOLD) {
    return true;
  }
  
  return false;
}

void calculateFiringAngles(int targetX, int targetY, int* panAngle, int* tiltAngle) {
  // Convert image coordinates to servo angles
  // This is a simplified calculation - you may need to calibrate this for your setup
  
  // Assuming camera resolution is 320x240 (QVGA)
  // Map X coordinate (0-320) to pan angle (PAN_MIN to PAN_MAX)
  *panAngle = map(targetX, 0, 320, PAN_MAX, PAN_MIN); // Inverted mapping
  
  // Map Y coordinate (0-240) to tilt angle (TILT_MIN to TILT_MAX)
  *tiltAngle = map(targetY, 0, 240, TILT_MIN, TILT_MAX);
  
  // Apply any offset correction based on calibration
  // *panAngle += PAN_OFFSET;
  // *tiltAngle += TILT_OFFSET;
  
  // Ensure angles are within valid range
  *panAngle = constrain(*panAngle, PAN_MIN, PAN_MAX);
  *tiltAngle = constrain(*tiltAngle, TILT_MIN, TILT_MAX);
}

void moveServos(int panAngle, int tiltAngle) {
  // Constrain angles to valid range
  panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
  tiltAngle = constrain(tiltAngle, TILT_MIN, TILT_MAX);
  
  // Update servo positions
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
  
  // Update current position variables
  currentPanPos = panAngle;
  currentTiltPos = tiltAngle;
  
  Serial.printf("Moved to position: Pan=%d, Tilt=%d\n", panAngle, tiltAngle);
  
  // Allow time for servos to reach position
  delay(200);
}

void fireAtTarget(int panAngle, int tiltAngle) {
  Serial.println("Firing sequence initiated");
  
  // Move to the target position
  moveServos(panAngle, tiltAngle);
  
  // Allow time for servos to reach position
  delay(500);
  
  // Fire the dart
  triggerServo.write(TRIGGER_FIRE);
  delay(300);
  triggerServo.write(TRIGGER_REST);
  
  Serial.println("Dart fired!");
  delay(1000); // Cool down period
}

long ultrasonicRead() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  long time = pulseIn(ECHO_PIN, HIGH);
  return time / 29 / 2; // Convert time to distance in cm
}

void checkSide() {
  stopMotors();
  delay(100);
  
  // Scan right
  scanServo.write(140);
  delay(300);
  distance_R = ultrasonicRead();
  Serial.print("Distance Right: "); Serial.println(distance_R);
  
  // Scan left
  scanServo.write(0);
  delay(500);
  distance_L = ultrasonicRead();
  Serial.print("Distance Left: "); Serial.println(distance_L);
  
  // Return to center
  scanServo.write(70);
  delay(300);
  
  // Compare distances and choose direction
  compareDistance();
}

void compareDistance() {
  if (distance_L > distance_R) {
    // More space on the left
    turnLeft();
    delay(500);
    forward();
    delay(600);
    turnRight();
    delay(500);
    forward();
    delay(600);
    turnRight();
    delay(400);
  } else {
    // More space on the right
    turnRight();
    delay(500);
    forward();
    delay(600);
    turnLeft();
    delay(500);
    forward();
    delay(600);
    turnLeft();
    delay(400);
  }
}

void forward() {
  digitalWrite(IN1, LOW);  // Left Motor forward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); // Right Motor forward
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, HIGH); // Left Motor backward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  // Right Motor backward
  digitalWrite(IN4, HIGH);
}

void turnRight() {
  digitalWrite(IN1, LOW);  // Left Motor forward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  // Right Motor backward
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, HIGH); // Left Motor backward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Right Motor forward
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
