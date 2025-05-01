# ESP32-S3 Autonomous Dart Turret Robot Wiring Diagram

## Components
- ESP32-S3 development board
- ESP32-CAM module
- L298N motor driver
- 2x IR sensors for line following
- HC-SR04 ultrasonic sensor
- 4x Servo motors (Pan, Tilt, Trigger, Scanning)
- 2x DC motors with wheels
- 5V power supply (capable of at least 3A)
- Logic level converter (3.3V to 5V) - optional but recommended
- Capacitors: 470μF and 100nF for power stabilization

## Connections

### Power
- Connect 5V and GND from power supply to all servo motors
- Connect 5V and GND to L298N motor driver
- Connect 5V and GND to ESP32-S3 (via voltage regulator if using higher voltage)
- Add 470μF capacitor between 5V and GND near the servos
- Add 100nF capacitor between 5V and GND near the ESP32-S3

### ESP32-S3 to Motor Driver (L298N)
- GPIO10 → ENA
- GPIO9 → IN1
- GPIO8 → IN2
- GPIO7 → IN3
- GPIO6 → IN4
- GPIO5 → ENB

### ESP32-S3 to Sensors
- GPIO39 → Left IR Sensor
- GPIO40 → Right IR Sensor
- GPIO41 → Echo Pin (Ultrasonic)
- GPIO42 → Trigger Pin (Ultrasonic)

### ESP32-S3 to Servos
- GPIO43 → Scanning Servo Signal
- GPIO12 → Pan Servo Signal
- GPIO13 → Tilt Servo Signal
- GPIO14 → Trigger Servo Signal

### ESP32-S3 to ESP32-CAM
- Connect ESP32-S3 to ESP32-CAM via I2C or UART
- If using I2C:
  - ESP32-S3 GPIO21 → ESP32-CAM SDA
  - ESP32-S3 GPIO22 → ESP32-CAM SCL
- If using UART:
  - ESP32-S3 GPIO17 → ESP32-CAM TX
  - ESP32-S3 GPIO18 → ESP32-CAM RX

## Notes
1. The ESP32-S3 has more GPIO pins than the Arduino, making it easier to connect all components
2. Ensure adequate power supply for all servos and motors to prevent brownouts
3. Keep wires to servos as short as possible to reduce noise
4. Consider adding optical isolation between ESP32-S3 and motor control signals for better reliability
5. The ESP32-CAM can be mounted on top of the robot with a clear view of potential targets
