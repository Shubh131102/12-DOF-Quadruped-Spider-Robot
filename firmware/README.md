# Firmware

Arduino firmware and ESP32 code for the 12-DOF quadruped spider robot.

## Structure
```
firmware/
├── arduino/
│   ├── main/
│   │   ├── main.ino
│   │   ├── config.h
│   │   ├── servo_control.cpp
│   │   ├── servo_control.h
│   │   ├── kinematics.cpp
│   │   ├── kinematics.h
│   │   ├── gait.cpp
│   │   ├── gait.h
│   │   └── communication.cpp
│   ├── libraries/
│   │   ├── Adafruit_PWMServoDriver/
│   │   └── ArduinoJson/
│   └── README.md
└── esp32/
    ├── camera_webserver/
    │   ├── camera_webserver.ino
    │   ├── camera_pins.h
    │   └── app_httpd.cpp
    └── README.md
```

## Overview

### Arduino Firmware

**Main Controller:** Arduino UNO R4 (or compatible)

**Responsibilities:**
- Servo control via PCA9685
- Inverse kinematics computation
- Gait generation and execution
- Serial communication with ESP32
- Command parsing and execution

**Key Components:**
- `main.ino`: Setup and main loop
- `servo_control`: PCA9685 PWM driver interface
- `kinematics`: Inverse kinematics solver
- `gait`: Gait pattern generator
- `communication`: Serial protocol handler

---

### ESP32 Firmware

**Module:** ESP32-CAM

**Responsibilities:**
- WiFi access point/station
- Camera video streaming
- Web server for control interface
- WebSocket communication
- Command relay to Arduino

---

## Arduino Firmware Details

### main.ino

Main program entry point and control loop.
```cpp
#include "config.h"
#include "servo_control.h"
#include "kinematics.h"
#include "gait.h"
#include "communication.h"

ServoController servos;
Kinematics ik;
GaitController gait;

void setup() {
  Serial.begin(115200);
  
  // Initialize PCA9685
  servos.begin();
  servos.setPWMFreq(50);  // 50Hz for servos
  
  // Load configuration
  loadConfig();
  
  // Initialize to standing pose
  gait.setStandingPose();
  
  Serial.println("Spider Robot Initialized");
}

void loop() {
  // Check for commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
  
  // Update gait
  gait.update();
  
  // Update servos
  servos.update();
  
  delay(20);  // 50Hz control loop
}
```

---

### config.h

Configuration constants and definitions.
```cpp
#ifndef CONFIG_H
#define CONFIG_H

// PCA9685 Configuration
#define PCA9685_ADDRESS 0x40
#define SERVO_FREQ 50  // Hz

// Servo channels
#define LF_HIP 0
#define LF_THIGH 1
#define LF_KNEE 2
#define RF_HIP 3
#define RF_THIGH 4
#define RF_KNEE 5
#define LH_HIP 6
#define LH_THIGH 7
#define LH_KNEE 8
#define RH_HIP 9
#define RH_THIGH 10
#define RH_KNEE 11

// Kinematics parameters (mm)
#define COXA_LENGTH 30
#define FEMUR_LENGTH 60
#define TIBIA_LENGTH 80

// Body dimensions (mm)
#define BODY_LENGTH 120
#define BODY_WIDTH 80

// PWM values
#define SERVO_MIN 150   // 0.5ms
#define SERVO_MAX 600   // 2.5ms
#define SERVO_NEUTRAL 307  // 1.5ms

// Control parameters
#define CONTROL_FREQUENCY 50  // Hz
#define INTERPOLATION_STEPS 10

// Safety limits
#define MAX_ANGLE_CHANGE 30  // degrees per update

#endif
```

---

### servo_control.h / servo_control.cpp

Servo control interface for PCA9685.
```cpp
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class ServoController {
public:
  ServoController();
  void begin();
  void setPWMFreq(float freq);
  void setAngle(uint8_t channel, float angle);
  void setAngles(float angles[12]);
  void update();
  
private:
  Adafruit_PWMServoDriver pwm;
  float currentAngles[12];
  float targetAngles[12];
  int interpolationStep;
  
  uint16_t angleToPulse(float angle);
};

#endif
```

**Implementation:**
```cpp
#include "servo_control.h"
#include "config.h"

ServoController::ServoController() : pwm(PCA9685_ADDRESS) {
  interpolationStep = 0;
  for (int i = 0; i < 12; i++) {
    currentAngles[i] = 90.0;
    targetAngles[i] = 90.0;
  }
}

void ServoController::begin() {
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
}

void ServoController::setAngle(uint8_t channel, float angle) {
  // Clamp angle to [0, 180]
  angle = constrain(angle, 0, 180);
  targetAngles[channel] = angle;
}

void ServoController::setAngles(float angles[12]) {
  for (int i = 0; i < 12; i++) {
    setAngle(i, angles[i]);
  }
}

void ServoController::update() {
  // Smooth interpolation to target angles
  for (int i = 0; i < 12; i++) {
    float diff = targetAngles[i] - currentAngles[i];
    
    // Apply max change limit
    if (abs(diff) > MAX_ANGLE_CHANGE) {
      diff = (diff > 0) ? MAX_ANGLE_CHANGE : -MAX_ANGLE_CHANGE;
    }
    
    currentAngles[i] += diff / INTERPOLATION_STEPS;
    
    // Set PWM
    uint16_t pulse = angleToPulse(currentAngles[i]);
    pwm.setPWM(i, 0, pulse);
  }
}

uint16_t ServoController::angleToPulse(float angle) {
  // Map angle [0, 180] to pulse [SERVO_MIN, SERVO_MAX]
  return map(angle * 100, 0, 18000, SERVO_MIN, SERVO_MAX);
}
```

---

### kinematics.h / kinematics.cpp

Inverse kinematics solver for leg positioning.
```cpp
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

struct Vector3 {
  float x, y, z;
};

struct LegAngles {
  float hip, thigh, knee;
};

class Kinematics {
public:
  Kinematics();
  bool solveIK(Vector3 footPos, LegAngles& angles);
  Vector3 solveFK(LegAngles angles);
  
private:
  float coxaLength;
  float femurLength;
  float tibiaLength;
};

#endif
```

**Implementation:**
```cpp
#include "kinematics.h"
#include "config.h"
#include <math.h>

Kinematics::Kinematics() {
  coxaLength = COXA_LENGTH;
  femurLength = FEMUR_LENGTH;
  tibiaLength = TIBIA_LENGTH;
}

bool Kinematics::solveIK(Vector3 footPos, LegAngles& angles) {
  float x = footPos.x;
  float y = footPos.y;
  float z = footPos.z;
  
  // Hip angle (yaw)
  angles.hip = atan2(y, x) * 180.0 / PI;
  
  // Distance in XY plane from hip joint
  float r = sqrt(x*x + y*y) - coxaLength;
  
  // Distance from hip to foot
  float D = sqrt(r*r + z*z);
  
  // Check if reachable
  if (D > (femurLength + tibiaLength)) {
    return false;  // Out of reach
  }
  
  // Knee angle (law of cosines)
  float cos_knee = (D*D - femurLength*femurLength - tibiaLength*tibiaLength) 
                   / (2 * femurLength * tibiaLength);
  
  if (cos_knee < -1.0 || cos_knee > 1.0) {
    return false;  // Invalid configuration
  }
  
  angles.knee = acos(cos_knee) * 180.0 / PI;
  
  // Thigh angle
  float alpha = atan2(z, r) * 180.0 / PI;
  float beta = acos((femurLength*femurLength + D*D - tibiaLength*tibiaLength) 
                    / (2 * femurLength * D)) * 180.0 / PI;
  
  angles.thigh = alpha + beta;
  
  return true;
}

Vector3 Kinematics::solveFK(LegAngles angles) {
  // Convert to radians
  float hip_rad = angles.hip * PI / 180.0;
  float thigh_rad = angles.thigh * PI / 180.0;
  float knee_rad = angles.knee * PI / 180.0;
  
  // Forward kinematics
  Vector3 pos;
  pos.x = (coxaLength + femurLength * cos(thigh_rad) 
          + tibiaLength * cos(thigh_rad + knee_rad)) * cos(hip_rad);
  pos.y = (coxaLength + femurLength * cos(thigh_rad) 
          + tibiaLength * cos(thigh_rad + knee_rad)) * sin(hip_rad);
  pos.z = femurLength * sin(thigh_rad) + tibiaLength * sin(thigh_rad + knee_rad);
  
  return pos;
}
```

---

### gait.h / gait.cpp

Gait pattern generation and execution.
```cpp
#ifndef GAIT_H
#define GAIT_H

#include "kinematics.h"

enum GaitType {
  GAIT_WALK,
  GAIT_TROT,
  GAIT_CRAWL
};

class GaitController {
public:
  GaitController();
  void setGait(GaitType type);
  void setSpeed(float speed);
  void setDirection(float forward, float turn);
  void setStandingPose();
  void update();
  
private:
  GaitType currentGait;
  float speed;
  float forwardVelocity;
  float turnVelocity;
  float gaitPhase;
  
  Kinematics ik;
  
  Vector3 getLegTarget(int legIndex, float phase);
  void updateLeg(int legIndex);
};

#endif
```

**Partial Implementation:**
```cpp
#include "gait.h"

GaitController::GaitController() {
  currentGait = GAIT_WALK;
  speed = 1.0;
  forwardVelocity = 0.0;
  turnVelocity = 0.0;
  gaitPhase = 0.0;
}

void GaitController::setStandingPose() {
  // Set all legs to neutral standing position
  Vector3 legPos = {100, 0, -80};  // mm
  
  for (int i = 0; i < 4; i++) {
    LegAngles angles;
    ik.solveIK(legPos, angles);
    
    // Apply to corresponding servos
    // Implementation depends on leg index to channel mapping
  }
}

void GaitController::update() {
  // Increment phase based on speed
  gaitPhase += 0.01 * speed;
  if (gaitPhase > 1.0) gaitPhase -= 1.0;
  
  // Update each leg based on gait pattern
  for (int i = 0; i < 4; i++) {
    updateLeg(i);
  }
}
```

---

## ESP32 Firmware Details

### camera_webserver.ino

Main ESP32-CAM program with web server and camera streaming.
```cpp
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// WiFi credentials
const char* ssid = "SpiderBot";
const char* password = "12345678";

// Function prototypes
void startCameraServer();
void setupCamera();

void setup() {
  Serial.begin(115200);
  
  // Initialize camera
  setupCamera();
  
  // Start WiFi AP
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Start web server
  startCameraServer();
  
  Serial.println("Camera Ready! Use 'http://" + IP.toString() + "' to connect");
}

void loop() {
  // Handle serial communication with Arduino
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    // Relay commands from web to Arduino
    Serial.println(cmd);
  }
  
  delay(10);
}
```

---

## Building and Uploading

### Arduino

**Requirements:**
- Arduino IDE 1.8.19 or higher (or Arduino IDE 2.x)
- Libraries:
  - Adafruit PWM Servo Driver Library
  - Wire (built-in)

**Steps:**
```bash
# Install Arduino IDE
# Install required libraries via Library Manager

# Open firmware/arduino/main/main.ino
# Select Board: Arduino UNO R4
# Select Port: /dev/ttyACM0 (Linux) or COM3 (Windows)
# Click Upload
```

---

### ESP32

**Requirements:**
- Arduino IDE with ESP32 board support
- ESP32 board package URL: 
  `https://dl.espressif.com/dl/package_esp32_index.json`

**Steps:**
```bash
# Add ESP32 board support in Arduino IDE
# Tools → Board → Boards Manager → ESP32
# Select Board: AI Thinker ESP32-CAM
# Select Port

# Upload using FTDI programmer:
# Connect: GND-GND, 5V-5V, TX-RX, RX-TX
# Bridge IO0 to GND for programming mode
# Click Upload
# Remove IO0-GND bridge after upload
# Press RESET button
```

---

## Serial Protocol

**Format:** `COMMAND:VALUE\n`

**Commands:**

| Command | Example | Description |
|---------|---------|-------------|
| GAIT | `GAIT:walk` | Set gait pattern |
| SPEED | `SPEED:1.5` | Set speed multiplier |
| MOVE | `MOVE:100,0` | Forward, turn values |
| POSE | `POSE:90,90,90` | Set joint angles |
| STOP | `STOP` | Emergency stop |
| STATUS | `STATUS` | Request status |

**Example:**
```
Arduino → ESP32: "STATUS:OK"
ESP32 → Arduino: "MOVE:50,10"
Arduino → ESP32: "ACK"
```

---

## Testing

### Servo Test
```cpp
// Test individual servo
void testServo(uint8_t channel) {
  servos.setAngle(channel, 0);
  delay(1000);
  servos.setAngle(channel, 90);
  delay(1000);
  servos.setAngle(channel, 180);
  delay(1000);
}
```

### IK Test
```cpp
// Test inverse kinematics
void testIK() {
  Vector3 target = {100, 0, -80};
  LegAngles angles;
  
  if (ik.solveIK(target, angles)) {
    Serial.println("IK Success:");
    Serial.print("Hip: "); Serial.println(angles.hip);
    Serial.print("Thigh: "); Serial.println(angles.thigh);
    Serial.print("Knee: "); Serial.println(angles.knee);
  } else {
    Serial.println("IK Failed: Out of reach");
  }
}
```

---

## Troubleshooting

**Servo not moving:**
- Check PCA9685 I2C address (default 0x40)
- Verify power supply (7.4V, sufficient current)
- Check servo channel mapping
- Test with simple servo sweep code

**Unstable movement:**
- Reduce speed multiplier
- Increase interpolation steps
- Check battery voltage
- Verify servo calibration

**ESP32 won't connect:**
- Verify WiFi credentials
- Check ESP32 is in AP mode
- Look for "SpiderBot" network
- Default IP: 192.168.4.1

---

## Notes

- Always test with robot suspended before ground testing
- Calibrate servos before first use
- Monitor servo temperatures during extended operation
- Keep emergency stop accessible
- Start with slow speeds and simple gaits
