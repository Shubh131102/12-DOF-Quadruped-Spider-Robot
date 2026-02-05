# Scripts

Utility scripts for testing, calibration, and robot operation.

## Structure
```
scripts/
├── calibration/
│   ├── calibrate_servos.py
│   ├── find_neutral.py
│   └── test_range.py
├── testing/
│   ├── test_servo.py
│   ├── test_ik.py
│   ├── test_gait.py
│   └── hardware_check.py
├── utilities/
│   ├── upload_firmware.sh
│   ├── monitor_serial.py
│   └── generate_poses.py
└── README.md
```

## Calibration Scripts

### calibrate_servos.py
Calibrate all 12 servos to find neutral (90°) position offsets.

**Usage:**
```bash
python scripts/calibration/calibrate_servos.py --port /dev/ttyACM0
```

**Process:**
1. Sets all servos to 90° PWM value
2. Visually inspect each leg
3. Record offset for each servo
4. Saves offsets to `config/control.yaml`

**Interactive prompts:**
```
Servo 0 (LF_hip) - Is it centered? (y/n/+/-): +
Offset: +2 degrees
Servo 1 (LF_thigh) - Is it centered? (y/n/+/-): -
Offset: -3 degrees
...
```

**Output:**
```yaml
# config/control.yaml - calibration section updated
calibration:
  LF_hip: 2
  LF_thigh: -3
  LF_knee: 0
  ...
```

---

### find_neutral.py
Automated script to find neutral positions using IMU feedback.

**Requirements:**
- MPU6050 IMU attached to robot
- Robot must be suspended

**Usage:**
```bash
python scripts/calibration/find_neutral.py --port /dev/ttyACM0 --imu
```

**Process:**
1. Reads IMU orientation
2. Adjusts servos iteratively
3. Finds level position automatically
4. Saves calibration values

---

### test_range.py
Test full range of motion for each servo.

**Usage:**
```bash
python scripts/calibration/test_range.py --port /dev/ttyACM0 --servo 0

# Or test all servos
python scripts/calibration/test_range.py --port /dev/ttyACM0 --all
```

**Output:**
```
Testing Servo 0 (LF_hip)
  0° - OK
  45° - OK
  90° - OK
  135° - OK
  180° - OK
Range test complete. Full range available.
```

---

## Testing Scripts

### test_servo.py
Test individual servo functionality.

**Usage:**
```bash
# Test specific servo
python scripts/testing/test_servo.py --port /dev/ttyACM0 --channel 0

# Sweep test
python scripts/testing/test_servo.py --port /dev/ttyACM0 --channel 0 --sweep

# Set specific angle
python scripts/testing/test_servo.py --port /dev/ttyACM0 --channel 0 --angle 90
```

**Example:**
```python
#!/usr/bin/env python3
import serial
import time
import argparse

def test_servo(port, channel, angle):
    """Test single servo at specific angle"""
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(2)
    
    command = f"SERVO:{channel},{angle}\n"
    ser.write(command.encode())
    print(f"Sent: {command.strip()}")
    
    response = ser.readline().decode().strip()
    print(f"Response: {response}")
    
    ser.close()

def sweep_servo(port, channel):
    """Sweep servo through full range"""
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(2)
    
    print(f"Sweeping servo {channel}")
    for angle in range(0, 181, 10):
        command = f"SERVO:{channel},{angle}\n"
        ser.write(command.encode())
        print(f"Angle: {angle}°")
        time.sleep(0.5)
    
    ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--channel', type=int, required=True)
    parser.add_argument('--angle', type=int, default=90)
    parser.add_argument('--sweep', action='store_true')
    
    args = parser.parse_args()
    
    if args.sweep:
        sweep_servo(args.port, args.channel)
    else:
        test_servo(args.port, args.channel, args.angle)
```

---

### test_ik.py
Test inverse kinematics calculations.

**Usage:**
```bash
python scripts/testing/test_ik.py --x 100 --y 0 --z -80

# Test workspace
python scripts/testing/test_ik.py --test-workspace
```

**Example:**
```python
#!/usr/bin/env python3
import numpy as np
import argparse

def inverse_kinematics(x, y, z, L1=30, L2=60, L3=80):
    """
    Calculate joint angles for desired foot position
    
    Args:
        x, y, z: Target position (mm)
        L1, L2, L3: Link lengths (mm)
    
    Returns:
        theta1, theta2, theta3: Joint angles (degrees)
    """
    # Hip angle
    theta1 = np.degrees(np.arctan2(y, x))
    
    # Distance in xy plane from hip joint
    r = np.sqrt(x**2 + y**2) - L1
    
    # Distance to foot
    D = np.sqrt(r**2 + z**2)
    
    # Check reachable
    if D > (L2 + L3):
        raise ValueError("Target out of reach")
    
    # Knee angle (law of cosines)
    cos_theta3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_theta3 = np.clip(cos_theta3, -1, 1)
    theta3 = np.degrees(np.arccos(cos_theta3))
    
    # Thigh angle
    alpha = np.degrees(np.arctan2(z, r))
    beta = np.degrees(np.arccos((L2**2 + D**2 - L3**2) / (2 * L2 * D)))
    theta2 = alpha + beta
    
    return theta1, theta2, theta3

def test_position(x, y, z):
    """Test IK for specific position"""
    try:
        angles = inverse_kinematics(x, y, z)
        print(f"Target: ({x}, {y}, {z})")
        print(f"Hip:   {angles[0]:.2f}°")
        print(f"Thigh: {angles[1]:.2f}°")
        print(f"Knee:  {angles[2]:.2f}°")
        return True
    except ValueError as e:
        print(f"Error: {e}")
        return False

def test_workspace():
    """Test IK across workspace"""
    print("Testing workspace...")
    successes = 0
    failures = 0
    
    for x in range(50, 151, 10):
        for z in range(-120, -39, 10):
            if test_position(x, 0, z):
                successes += 1
            else:
                failures += 1
    
    print(f"\nResults: {successes} successes, {failures} failures")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=100)
    parser.add_argument('--y', type=float, default=0)
    parser.add_argument('--z', type=float, default=-80)
    parser.add_argument('--test-workspace', action='store_true')
    
    args = parser.parse_args()
    
    if args.test_workspace:
        test_workspace()
    else:
        test_position(args.x, args.y, args.z)
```

---

### test_gait.py
Test gait patterns without full system.

**Usage:**
```bash
python scripts/testing/test_gait.py --gait walk --duration 10

# Visualize gait pattern
python scripts/testing/test_gait.py --gait trot --visualize
```

---

### hardware_check.py
Comprehensive hardware diagnostics.

**Usage:**
```bash
python scripts/testing/hardware_check.py --port /dev/ttyACM0
```

**Checks:**
- [ ] Serial connection
- [ ] PCA9685 I2C communication
- [ ] All 12 servos responding
- [ ] ESP32-CAM connectivity
- [ ] Battery voltage
- [ ] Servo current draw

**Output:**
```
Hardware Check
==============
✓ Serial connection: OK
✓ PCA9685 found at 0x40
✓ Servo 0-11: All responding
✓ ESP32-CAM: Reachable at 192.168.4.1
✓ Battery: 7.3V (OK)
⚠ Current draw: 1850mA (High)

All systems operational.
```

---

## Utility Scripts

### upload_firmware.sh
Automated firmware upload script.

**Usage:**
```bash
# Upload Arduino firmware
./scripts/utilities/upload_firmware.sh arduino

# Upload ESP32 firmware
./scripts/utilities/upload_firmware.sh esp32
```

**Script:**
```bash
#!/bin/bash

if [ "$1" == "arduino" ]; then
    echo "Uploading Arduino firmware..."
    arduino-cli compile --fqbn arduino:avr:uno firmware/arduino/main
    arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno firmware/arduino/main
    echo "Upload complete!"
    
elif [ "$1" == "esp32" ]; then
    echo "Uploading ESP32 firmware..."
    arduino-cli compile --fqbn esp32:esp32:esp32cam firmware/esp32/camera_webserver
    arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32cam firmware/esp32/camera_webserver
    echo "Upload complete!"
    
else
    echo "Usage: ./upload_firmware.sh [arduino|esp32]"
fi
```

---

### monitor_serial.py
Monitor serial output with timestamp.

**Usage:**
```bash
python scripts/utilities/monitor_serial.py --port /dev/ttyACM0
```

**Features:**
- Timestamped output
- Color-coded messages (errors in red)
- Save to log file
- Filter by keyword

---

### generate_poses.py
Generate custom pose configurations.

**Usage:**
```bash
# Generate sitting pose
python scripts/utilities/generate_poses.py --type sit --output config/poses/sit.yaml

# Generate random pose (for testing)
python scripts/utilities/generate_poses.py --random --safe
```

---

## Complete Workflow Examples

### Initial Setup and Calibration
```bash
# 1. Check hardware
python scripts/testing/hardware_check.py --port /dev/ttyACM0

# 2. Test each servo
for i in {0..11}; do
    python scripts/testing/test_servo.py --port /dev/ttyACM0 --channel $i --sweep
done

# 3. Calibrate servos
python scripts/calibration/calibrate_servos.py --port /dev/ttyACM0

# 4. Test IK
python scripts/testing/test_ik.py --test-workspace

# 5. Test gait
python scripts/testing/test_gait.py --gait walk --duration 5
```

---

### Daily Testing Routine
```bash
# Quick hardware check
python scripts/testing/hardware_check.py --port /dev/ttyACM0

# Test standing pose
python scripts/testing/test_servo.py --port /dev/ttyACM0 --preset standing

# Monitor for 30 seconds
timeout 30 python scripts/utilities/monitor_serial.py --port /dev/ttyACM0
```

---

### Debugging Workflow
```bash
# 1. Monitor serial output
python scripts/utilities/monitor_serial.py --port /dev/ttyACM0 --log debug.log

# 2. Test specific servo
python scripts/testing/test_servo.py --port /dev/ttyACM0 --channel 5 --angle 90

# 3. Check IK for problematic position
python scripts/testing/test_ik.py --x 120 --y 20 --z -90

# 4. Full hardware diagnostic
python scripts/testing/hardware_check.py --port /dev/ttyACM0 --verbose
```

---

## Requirements

**Python packages:**
```
pyserial>=3.5
numpy>=1.19
pyyaml>=5.4
matplotlib>=3.3 (for visualization)
```

Install:
```bash
pip install -r scripts/requirements.txt
```

---

## Creating Custom Scripts

**Template:**
```python
#!/usr/bin/env python3
"""
Script description here
"""

import serial
import time
import argparse

def main(args):
    """Main function"""
    ser = serial.Serial(args.port, 115200, timeout=1)
    time.sleep(2)
    
    # Your code here
    
    ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script description")
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    # Add more arguments
    
    args = parser.parse_args()
    main(args)
```

---

## Troubleshooting

### Permission Denied on Serial Port

**Linux:**
```bash
sudo chmod 666 /dev/ttyACM0
# Or permanently:
sudo usermod -aG dialout $USER
```

### Script Not Finding Port
```bash
# List all ports
python -m serial.tools.list_ports

# On Linux
ls /dev/tty*

# On Windows
# Check Device Manager
```

### Import Errors
```bash
# Install missing packages
pip install pyserial numpy pyyaml

# Or use requirements file
pip install -r scripts/requirements.txt
```

---

## Notes

- Always test with robot suspended before ground testing
- Keep backup of calibration values
- Use `--help` flag on any script for detailed usage
- Scripts assume default serial settings (115200 baud)
- Some scripts require Arduino firmware to support specific commands
