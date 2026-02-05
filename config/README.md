# Configuration Files

Configuration parameters for the 12-DOF quadruped spider robot.

## Files

### servos.yaml
Servo configuration for PCA9685 PWM driver and channel mapping.
```yaml
i2c_addr: 0x40        # PCA9685 I2C address
neutral_pwm: 307      # Neutral position (~1.5ms pulse at 50Hz)

channels:
  # Left Front leg
  LF_hip:   0         # Channel 0 - Hip yaw rotation
  LF_thigh: 1         # Channel 1 - Thigh pitch
  LF_knee:  2         # Channel 2 - Knee pitch
  
  # Right Front leg
  RF_hip:   3         # Channel 3
  RF_thigh: 4         # Channel 4
  RF_knee:  5         # Channel 5
  
  # Left Hind leg
  LH_hip:   6         # Channel 6
  LH_thigh: 7         # Channel 7
  LH_knee:  8         # Channel 8
  
  # Right Hind leg
  RH_hip:   9         # Channel 9
  RH_thigh: 10        # Channel 10
  RH_knee:  11        # Channel 11
```

---

### gait_params.yaml
Gait pattern parameters for different walking modes.
```yaml
gaits:
  walk:
    step_height: 30        # mm
    stride_length: 50      # mm
    duty_cycle: 0.75       # Percentage of cycle foot is on ground
    phase_offset: [0, 0.5, 0.75, 0.25]  # LF, RF, LH, RH
    speed: 1.0             # Speed multiplier
  
  trot:
    step_height: 40
    stride_length: 60
    duty_cycle: 0.5
    phase_offset: [0, 0.5, 0.5, 0]
    speed: 1.5
  
  crawl:
    step_height: 20
    stride_length: 30
    duty_cycle: 0.9
    phase_offset: [0, 0.25, 0.5, 0.75]
    speed: 0.5

default_gait: "walk"
```

---

### kinematics.yaml
Robot kinematic parameters and dimensions.
```yaml
# Link lengths (mm)
dimensions:
  coxa_length: 30       # Hip to thigh joint
  femur_length: 60      # Thigh length
  tibia_length: 80      # Lower leg length
  
  body_length: 120      # Front to back
  body_width: 80        # Side to side

# Leg mounting positions (mm from body center)
leg_positions:
  LF: [60, 40, 0]       # x, y, z
  RF: [60, -40, 0]
  LH: [-60, 40, 0]
  RH: [-60, -40, 0]

# Joint limits (degrees)
joint_limits:
  hip:
    min: -45
    max: 45
  thigh:
    min: 30
    max: 150
  knee:
    min: 30
    max: 150

# Default standing pose (degrees)
default_pose:
  hip: 0
  thigh: 90
  knee: 90

# Workspace limits
workspace:
  min_height: 40        # mm above ground
  max_height: 120       # mm above ground
  max_reach: 150        # mm radial distance
```

---

### control.yaml
Control system parameters.
```yaml
# PWM configuration
pwm:
  frequency: 50         # Hz (standard servo frequency)
  min_pulse: 150        # ~0.5ms (0 degrees)
  max_pulse: 600        # ~2.5ms (180 degrees)
  neutral_pulse: 307    # ~1.5ms (90 degrees)

# Servo calibration offsets (degrees)
# Adjust these values per servo for neutral position
calibration:
  LF_hip: 0
  LF_thigh: 2
  LF_knee: -3
  RF_hip: 1
  RF_thigh: -2
  RF_knee: 0
  LH_hip: -1
  LH_thigh: 3
  LH_knee: 2
  RH_hip: 0
  RH_thigh: -1
  RH_knee: 1

# Control loop settings
control:
  update_rate: 50       # Hz
  interpolation_steps: 10  # Steps between poses
  
# Safety limits
safety:
  max_angle_change: 30  # degrees per update
  emergency_stop_height: 20  # mm (too low)
  max_current: 2000     # mA total draw
```

---

### wifi.yaml
WiFi and network configuration for ESP32-CAM.
```yaml
wifi:
  ssid: "SpiderBot"
  password: "12345678"
  mode: "AP"            # Access Point mode
  channel: 6
  
  # Station mode (connect to existing network)
  # mode: "STA"
  # router_ssid: "YourNetwork"
  # router_password: "YourPassword"

network:
  ip: "192.168.4.1"
  gateway: "192.168.4.1"
  subnet: "255.255.255.0"
  
camera:
  resolution: "VGA"     # VGA, SVGA, XGA
  quality: 12           # 0-63 (lower = better quality)
  framerate: 10         # fps
  
control_server:
  port: 80
  websocket_port: 81
```

---

## Configuration Usage

### Loading Configuration

**Arduino/C++:**
```cpp
#include <ArduinoJson.h>

void loadServoConfig() {
  File file = SPIFFS.open("/config/servos.yaml", "r");
  // Parse YAML and configure servos
}
```

**Python:**
```python
import yaml

with open('config/servos.yaml', 'r') as f:
    config = yaml.safe_load(f)

i2c_addr = config['i2c_addr']
channels = config['channels']
```

---

### Servo Channel Mapping

**Leg Naming Convention:**
- **LF:** Left Front
- **RF:** Right Front
- **LH:** Left Hind
- **RH:** Right Hind

**Joint Names:**
- **hip:** Vertical rotation (yaw)
- **thigh:** Forward/backward (pitch)
- **knee:** Up/down (pitch)

**PCA9685 Channels (0-11):**
```
Leg    Hip  Thigh  Knee
LF:     0     1      2
RF:     3     4      5
LH:     6     7      8
RH:     9    10     11
```

---

### PWM Pulse Width Calculation

For MG90S servos at 50Hz:

**Pulse width to angle:**
```
angle = 0°   → pulse = 150 (0.5ms)
angle = 90°  → pulse = 307 (1.5ms)
angle = 180° → pulse = 600 (2.5ms)
```

**Formula:**
```
pulse = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
pulse = 150 + (angle / 180.0) * (600 - 150)
pulse = 150 + (angle * 2.5)
```

---

### Calibration Procedure

1. **Center all servos:**
```bash
python scripts/calibrate_servos.py --center
```

2. **Adjust offsets in control.yaml:**
- Observe which servos are off-center
- Update calibration offsets
- Positive offset = rotate clockwise
- Negative offset = rotate counter-clockwise

3. **Test standing pose:**
```bash
python scripts/test_pose.py --pose standing
```

4. **Fine-tune:**
- Adjust until all legs are symmetrical
- Save final calibration values

---

### Gait Selection

**Walk (default):**
- Stable, energy efficient
- Good for flat terrain
- 3 legs always on ground

**Trot:**
- Faster movement
- Diagonal leg pairs move together
- Less stable than walk

**Crawl:**
- Maximum stability
- Very slow
- Good for rough terrain
- Nearly all legs on ground at once

**Change gait:**
```python
robot.set_gait('trot')
robot.set_speed(1.5)
```

---

### Inverse Kinematics Parameters

**Dimensions used in IK calculations:**
```
Total leg reach = coxa + femur + tibia
                = 30 + 60 + 80 = 170mm

Workspace radius ≈ 150mm (accounting for joint limits)
Standing height range: 40-120mm
```

**Default standing position:**
- Hip: 0° (straight forward)
- Thigh: 90° (perpendicular to body)
- Knee: 90° (perpendicular to thigh)
- Foot position: ~100mm below body

---

## Customization Guide

### Adjusting Leg Length

If you modify CAD files and change leg lengths:

1. Update `kinematics.yaml`:
```yaml
dimensions:
  coxa_length: 35      # Changed from 30
  femur_length: 70     # Changed from 60
  tibia_length: 90     # Changed from 80
```

2. Recalibrate inverse kinematics
3. Test workspace limits
4. Adjust gait parameters if needed

---

### Changing Servo Type

If using different servos (e.g., MG996R):

1. Update PWM range in `control.yaml`:
```yaml
pwm:
  min_pulse: 100       # Check servo datasheet
  max_pulse: 500
  neutral_pulse: 300
```

2. Test and recalibrate
3. May need to adjust control loop rate

---

### Adding New Gaits

Create custom gait in `gait_params.yaml`:
```yaml
gaits:
  custom_gait:
    step_height: 35
    stride_length: 45
    duty_cycle: 0.6
    phase_offset: [0, 0.33, 0.66, 1.0]
    speed: 1.2
```

Test with:
```python
robot.set_gait('custom_gait')
```

---

## Troubleshooting

### Servo Not Responding

**Check:**
1. Correct channel in `servos.yaml`
2. I2C address (default 0x40)
3. Power supply (7.4V, sufficient current)
4. PWM frequency (50Hz)

**Debug:**
```python
# Test individual servo
python scripts/test_servo.py --channel 0 --angle 90
```

---

### Robot Not Standing Level

**Fix:**
1. Check calibration offsets in `control.yaml`
2. Measure actual servo positions
3. Adjust offsets until level
4. Save and test

---

### Jerky Movement

**Possible causes:**
- `interpolation_steps` too low (increase to 20)
- `update_rate` too low (increase to 100Hz if possible)
- Power supply voltage dropping
- Servo quality/wear

---

### Gait Not Working

**Check:**
1. Gait name spelled correctly
2. Phase offsets sum to valid pattern
3. Step height within workspace limits
4. Stride length achievable with IK

---

## Safety Notes

- Always test with robot suspended first
- Start with slow speeds
- Keep emergency stop accessible
- Monitor servo temperatures
- Check battery voltage regularly
- Never exceed joint limits in config

---

## Default Configuration Summary

| Parameter | Value | Unit |
|-----------|-------|------|
| PWM Frequency | 50 | Hz |
| Neutral Pulse | 307 | counts |
| Control Rate | 50 | Hz |
| Default Gait | walk | - |
| Standing Height | 100 | mm |
| Max Reach | 150 | mm |
| Total DOF | 12 | - |

---

## Notes

- All angles in degrees unless specified
- All lengths in millimeters
- Configuration files use YAML format
- Calibration values are robot-specific
- Save original config before modifications
