# Launch Files

ROS2 launch files for the 12-DOF quadruped spider robot.

## Available Launch Files

### bringup.launch.py
**Purpose:** Launch complete robot system

**Launched Nodes:**
- `cam_relay`: ESP32-CAM video streaming interface
- `servo_bridge`: Arduino/PCA9685 servo control bridge
- `gait_controller`: Gait pattern generation
- `kinematics_node`: Inverse kinematics solver
- `teleop_keyboard`: Keyboard teleoperation (optional)
- `robot_state_publisher`: URDF-based robot model
- `joint_state_publisher`: Servo position publishing

**Arguments:**
```bash
use_sim_time      Use simulation time (default: false)
camera_enabled    Enable ESP32-CAM (default: true)
teleop_enabled    Enable keyboard control (default: true)
i2c_addr          PCA9685 I2C address (default: 0x40)
```

**Usage:**
```bash
# Basic launch
ros2 launch spider_robot bringup.launch.py

# Without camera
ros2 launch spider_robot bringup.launch.py camera_enabled:=false

# With custom I2C address
ros2 launch spider_robot bringup.launch.py i2c_addr:=0x41
```

---

### teleop.launch.py
**Purpose:** Launch teleoperation only (assumes robot already running)

**Usage:**
```bash
ros2 launch spider_robot teleop.launch.py
```

---

### visualization.launch.py
**Purpose:** Launch RViz visualization with robot model

**Usage:**
```bash
ros2 launch spider_robot visualization.launch.py
```

---

## Node Communication
```
┌──────────────┐
│ ESP32-CAM    │
└──────┬───────┘
       │ HTTP Stream
┌──────▼───────┐
│  cam_relay   │ → /spider/camera (sensor_msgs/Image)
└──────────────┘

┌──────────────┐
│   Arduino    │
└──────┬───────┘
       │ Serial
┌──────▼───────────┐
│  servo_bridge    │ ← /spider/joint_commands
└──────┬───────────┘
       │
       └─→ /spider/joint_states

┌──────────────────┐
│ teleop_keyboard  │ → /spider/cmd_vel
└──────────────────┘
       │
       ▼
┌──────────────────┐
│ gait_controller  │ → /spider/foot_targets
└──────────────────┘
       │
       ▼
┌──────────────────┐
│ kinematics_node  │ → /spider/joint_commands
└──────────────────┘
```

---

## Topics

### Published Topics

**Camera:**
- `/spider/camera` (sensor_msgs/Image): Camera stream from ESP32-CAM

**Control:**
- `/spider/joint_states` (sensor_msgs/JointState): Current servo positions
- `/spider/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/spider/gait_state` (std_msgs/String): Current gait pattern

**Visualization:**
- `/tf` (tf2_msgs/TFMessage): Transform tree
- `/robot_description` (std_msgs/String): URDF model

### Subscribed Topics

**Control:**
- `/spider/cmd_vel` (geometry_msgs/Twist): Input velocity commands
- `/spider/joint_commands` (sensor_msgs/JointState): Target servo positions
- `/spider/gait_select` (std_msgs/String): Gait pattern selection

---

## Parameters

### servo_bridge

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| i2c_addr | string | 0x40 | PCA9685 I2C address |
| serial_port | string | /dev/ttyACM0 | Arduino serial port |
| baud_rate | int | 115200 | Serial baud rate |
| control_frequency | int | 50 | Control loop Hz |

### cam_relay

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| camera_url | string | http://192.168.4.1:81/stream | ESP32-CAM stream URL |
| frame_rate | int | 10 | Target FPS |
| camera_topic | string | /spider/camera | Output topic |

### gait_controller

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| gait_type | string | walk | Initial gait (walk/trot/crawl) |
| speed | float | 1.0 | Speed multiplier |
| step_height | float | 30.0 | Step height (mm) |

---

## Usage Examples

### Basic Operation
```bash
# Terminal 1: Launch robot
ros2 launch spider_robot bringup.launch.py

# Terminal 2: Control with keyboard
# Use arrow keys for movement
# q/z for speed up/down
```

---

### Manual Servo Control
```bash
# Launch robot
ros2 launch spider_robot bringup.launch.py

# Publish joint command
ros2 topic pub /spider/joint_commands sensor_msgs/JointState \
  "{name: ['LF_hip', 'LF_thigh', 'LF_knee'], \
    position: [0.0, 1.57, 1.57]}"
```

---

### Change Gait
```bash
# Select trot gait
ros2 topic pub /spider/gait_select std_msgs/String "data: 'trot'"

# Select crawl gait
ros2 topic pub /spider/gait_select std_msgs/String "data: 'crawl'"
```

---

### View Camera Stream
```bash
# Launch with camera
ros2 launch spider_robot bringup.launch.py

# View in RViz or rqt_image_view
ros2 run rqt_image_view rqt_image_view /spider/camera
```

---

### Visualization in RViz
```bash
# Terminal 1: Launch robot
ros2 launch spider_robot bringup.launch.py

# Terminal 2: Launch RViz
ros2 launch spider_robot visualization.launch.py
```

---

## Troubleshooting

### Servo Bridge Not Starting

**Check Arduino connection:**
```bash
ls /dev/ttyACM*
# Should show /dev/ttyACM0 or similar

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Or add user to dialout group
sudo usermod -aG dialout $USER
```

**Verify serial port:**
```bash
# Test serial connection
screen /dev/ttyACM0 115200

# Or use Arduino IDE Serial Monitor
```

---

### Camera Not Streaming

**Check ESP32 connection:**
```bash
# Ping ESP32
ping 192.168.4.1

# Check camera stream URL in browser
firefox http://192.168.4.1:81/stream
```

**Verify WiFi connection:**
```bash
# List WiFi networks
nmcli device wifi list

# Connect to SpiderBot
nmcli device wifi connect SpiderBot password 12345678
```

---

### Nodes Not Communicating

**Check topics:**
```bash
ros2 topic list
ros2 topic info /spider/cmd_vel
ros2 topic hz /spider/joint_states
```

**Check node status:**
```bash
ros2 node list
ros2 node info /servo_bridge
```

**View TF tree:**
```bash
ros2 run tf2_tools view_frames
```

---

## Hardware Setup

**Before launching:**

1. Connect Arduino to PC via USB
2. Power on robot (7.4V LiPo + BEC)
3. Ensure ESP32-CAM WiFi is active
4. Connect to "SpiderBot" WiFi network
5. Verify all connections:
   - Arduino ↔ PCA9685 (I2C)
   - Arduino ↔ ESP32 (Serial)
   - PCA9685 ↔ 12 Servos

---

## Safety Checklist

Before running:
- [ ] Robot is suspended or on soft surface
- [ ] Battery fully charged
- [ ] All servo connections secure
- [ ] Emergency stop accessible
- [ ] Workspace clear of obstacles
- [ ] Camera (if enabled) not pointing at bright lights

---

## Performance Tips

**For smoother control:**
```yaml
# Increase control frequency
control_frequency: 100  # Hz

# More interpolation steps
interpolation_steps: 20
```

**For lower latency:**
```yaml
# Reduce camera frame rate
frame_rate: 5  # FPS

# Simplify gait
gait_type: "crawl"  # Slower but stable
```

---

## Notes

- Always launch with robot suspended for first test
- Use emergency stop if robot behaves unexpectedly
- Monitor servo temperatures during operation
- Battery life: ~30-60 minutes depending on usage
- Default WiFi: SSID "SpiderBot", Password "12345678"
