# Documentation

Project documentation, presentations, and technical reports for the 12-DOF quadruped spider robot.

## Contents

**presentation.pdf** (or .pptx)
- Project overview and demonstration
- Design process and iterations
- Inverse kinematics explanation
- Gait implementation
- Hardware and software architecture
- Demo videos and results

**Technical_Report.pdf**
- Complete project documentation
- Mechanical design details
- Inverse kinematics derivation
- Gait planning algorithms
- Control system implementation
- Test results and performance analysis

**Assembly_Instructions.pdf**
- Step-by-step assembly guide
- Wiring diagrams
- Servo installation
- Electronics mounting
- Testing procedures

**User_Manual.pdf**
- Quick start guide
- Operation instructions
- Gait selection
- WiFi control interface
- Troubleshooting
- Maintenance

## Project Overview

**12-DOF Quadruped Spider Robot**
- 4 legs with 3 DOF each (hip, thigh, knee)
- Arduino-based control system
- PCA9685 PWM servo driver
- ESP32-CAM for vision and WiFi control
- Custom inverse kinematics
- Multiple gait patterns

## Key Features

**Hardware:**
- 12x MG90S servos (or equivalent)
- PCA9685 16-channel PWM driver
- Arduino UNO R4 controller
- ESP32-CAM module
- 7.4V LiPo battery with BEC
- 3D printed chassis and legs

**Software:**
- Custom inverse kinematics solver
- Multi-gait locomotion (walk, trot, crawl)
- WiFi remote control interface
- Real-time video streaming
- Python GUI for testing

**Control Modes:**
- Manual control (joystick/keyboard)
- Autonomous navigation
- WiFi web interface
- Python API

## Inverse Kinematics

### Mathematical Model

Each leg has 3 joints with the following kinematic chain:
```
Body → Hip (θ₁, yaw) → Thigh (θ₂, pitch) → Knee (θ₃, pitch) → Foot
```

**Link Lengths:**
- L₁ = 30mm (coxa)
- L₂ = 60mm (femur)
- L₃ = 80mm (tibia)

**Forward Kinematics:**
```
x = L₁ + (L₂·cos(θ₂) + L₃·cos(θ₂+θ₃))·cos(θ₁)
y = (L₁ + L₂·cos(θ₂) + L₃·cos(θ₂+θ₃))·sin(θ₁)
z = L₂·sin(θ₂) + L₃·sin(θ₂+θ₃)
```

**Inverse Kinematics Solution:**
```
θ₁ = atan2(y, x)
θ₃ = acos((x² + y² + z² - L₁² - L₂² - L₃²) / (2·L₂·L₃))
θ₂ = atan2(z, √(x²+y²)) - atan2(L₃·sin(θ₃), L₂+L₃·cos(θ₃))
```

**Workspace:**
- Radial reach: ~150mm
- Height range: 40-120mm
- Angular range: ±45° hip rotation

### Implementation
```python
def inverse_kinematics(x, y, z):
    """
    Compute joint angles for desired foot position.
    
    Args:
        x, y, z: Target foot position (mm)
    
    Returns:
        theta1, theta2, theta3: Joint angles (radians)
    """
    L1, L2, L3 = 30, 60, 80  # Link lengths
    
    # Hip angle
    theta1 = math.atan2(y, x)
    
    # Distance in xy plane
    r = math.sqrt(x**2 + y**2) - L1
    
    # Distance to foot
    D = math.sqrt(r**2 + z**2)
    
    # Knee angle (law of cosines)
    cos_theta3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta3 = math.acos(np.clip(cos_theta3, -1, 1))
    
    # Thigh angle
    alpha = math.atan2(z, r)
    beta = math.acos((L2**2 + D**2 - L3**2) / (2 * L2 * D))
    theta2 = alpha + beta
    
    return theta1, theta2, theta3
```

## Gait Patterns

### Walk Gait

**Characteristics:**
- 3 legs on ground at all times
- Maximum stability
- Phase offsets: [0, 0.5, 0.75, 0.25]
- Duty cycle: 75%

**Sequence:**
```
Step 1: LF lifts, RF/LH/RH support
Step 2: RF lifts, LF/LH/RH support
Step 3: LH lifts, LF/RF/RH support
Step 4: RH lifts, LF/RF/LH support
```

### Trot Gait

**Characteristics:**
- Diagonal leg pairs move together
- Faster than walk
- Phase offsets: [0, 0.5, 0.5, 0]
- Duty cycle: 50%

**Pairs:**
- Pair 1: LF + RH
- Pair 2: RF + LH

### Crawl Gait

**Characteristics:**
- One leg moves at a time
- Maximum stability
- Very slow
- Phase offsets: [0, 0.25, 0.5, 0.75]
- Duty cycle: 90%

## System Architecture
```
┌─────────────┐
│   ESP32-CAM │ ← WiFi Control
│   (Camera)  │
└──────┬──────┘
       │ Serial
┌──────▼──────────┐
│  Arduino UNO R4 │
│   (Main CPU)    │
└──────┬──────────┘
       │ I2C
┌──────▼──────────┐
│    PCA9685      │ ← 12 Servos
│  (PWM Driver)   │
└─────────────────┘
```

**Data Flow:**
1. User command (WiFi/Serial)
2. Arduino processes gait/IK
3. Calculate joint angles
4. Send PWM signals via PCA9685
5. Servos move to positions
6. Camera streams video back

## Performance Specifications

| Metric | Value |
|--------|-------|
| Walking Speed | 5-10 cm/s |
| Step Height | 20-40 mm |
| Stride Length | 30-60 mm |
| Control Frequency | 50 Hz |
| IK Computation | <1ms |
| Gait Update | 20ms |
| Total Weight | ~850g |
| Battery Life | 30-60 min |

## Control Interface

### WiFi Web Interface

**Features:**
- Joystick control (forward/backward/turn)
- Gait selection dropdown
- Speed adjustment slider
- Live camera feed
- Battery voltage monitor
- Emergency stop button

**Access:**
```
1. Connect to "SpiderBot" WiFi
2. Open browser: http://192.168.4.1
3. Use on-screen controls
```

### Python GUI

**Features:**
- Servo testing (individual control)
- IK testing (foot position input)
- Gait visualization
- Pose editor
- Calibration utility

**Usage:**
```bash
python python_gui/control_panel.py
```

### Serial Commands

**Protocol:**
```
Command format: CMD:VALUE\n

Examples:
GAIT:walk       - Set walk gait
SPEED:1.5       - Set speed multiplier
MOVE:100,0      - Move forward 100, turn 0
POSE:90,90,90   - Set all joints to 90°
STOP            - Emergency stop
```

## Bill of Materials

From `bom/bom.csv`:

| Part | Quantity | Notes |
|------|----------|-------|
| MG90S Servo | 12 | Or equivalent 9g servo |
| PCA9685 PWM Driver | 1 | 16-channel I2C |
| Arduino UNO R4 | 1 | Or compatible controller |
| ESP32-CAM | 1 | Optional for vision |
| Power Distribution Board | 1 | Custom PCB |
| 7.4V LiPo Battery | 1 | 1000-2000mAh |
| BEC (5V/3A) | 1 | For logic power |
| 3D Printed Parts | - | See CAD folder |
| Screws/Standoffs | - | M2, M3 hardware |

**Estimated Cost:** ~$80-100 USD

## Assembly Time

- 3D Printing: ~38 hours
- Assembly: 4-6 hours
- Wiring: 2-3 hours
- Programming: 1-2 hours
- Testing/Calibration: 2-4 hours

**Total:** ~50-55 hours

## Testing Results

**IK Accuracy:**
- Position error: <2mm average
- Joint angle error: <2° average
- Computation time: 0.8ms per leg

**Gait Performance:**
- Walk: Stable on flat surfaces
- Trot: Stable at speeds <15 cm/s
- Crawl: Stable on 15° inclines

**Battery Life:**
- Walking: ~45 minutes
- Standing idle: ~90 minutes
- With camera: ~30 minutes

## Future Improvements

- [ ] IMU integration for balance control
- [ ] Ultrasonic sensors for obstacle avoidance
- [ ] Autonomous navigation algorithms
- [ ] Machine learning for adaptive gaits
- [ ] Carbon fiber legs for weight reduction
- [ ] LiDAR sensor for mapping

## Related Projects

- [Spot Micro](https://github.com/mike4192/spotMicro) - Inspired design
- [Stanford Doggo](https://github.com/Nate711/StanfordDoggoProject) - Advanced quadruped
- [PhantomX](http://www.trossenrobotics.com/phantomx-ax-quadruped.aspx) - Commercial version

## References

1. "Quadruped Gait Generation and Control" - Stanford Robotics Lab
2. "Inverse Kinematics for Legged Robots" - MIT OpenCourseWare
3. "Modern Robotics" by Kevin Lynch
4. PCA9685 Datasheet - NXP Semiconductors
5. MG90S Servo Datasheet - TowerPro

## Citation
```bibtex
@project{jangle2024quadruped,
  title={12-DOF Quadruped Spider Robot with WiFi Control},
  author={Jangle, Shubham},
  year={2024},
  note={Personal robotics project}
}
```

## Contact

Shubham Jangle  
Email: sjang041@ucr.edu  
GitHub: [github.com/Shubh131102](https://github.com/Shubh131102)

## License

See LICENSE file in repository root.
