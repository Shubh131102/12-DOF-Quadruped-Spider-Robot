# Source Code

ROS2 packages and Arduino libraries for the 12-DOF quadruped spider robot.

## Structure
```
src/
├── spider_control/          ROS2 control package
│   ├── spider_control/
│   │   ├── __init__.py
│   │   ├── servo_bridge.py
│   │   ├── gait_controller.py
│   │   ├── kinematics_node.py
│   │   └── trajectory_planner.py
│   ├── package.xml
│   └── setup.py
├── spider_cam/              ROS2 camera package
│   ├── spider_cam/
│   │   ├── __init__.py
│   │   ├── cam_relay.py
│   │   └── image_processor.py
│   ├── package.xml
│   └── setup.py
└── arduino_libraries/       Custom Arduino libraries
    ├── SpiderKinematics/
    ├── GaitGenerator/
    └── ServoController/
```

## ROS2 Packages

### spider_control
Main control package for robot locomotion.

**Nodes:**

**servo_bridge**
- Interfaces with Arduino via serial
- Publishes joint states
- Subscribes to joint commands
- Handles emergency stop

**Subscribed Topics:**
- `/spider/joint_commands` (sensor_msgs/JointState)
- `/spider/emergency_stop` (std_msgs/Bool)

**Published Topics:**
- `/spider/joint_states` (sensor_msgs/JointState)
- `/spider/servo_status` (custom_msgs/ServoStatus)

**Parameters:**
- `serial_port`: Arduino serial port (default: /dev/ttyACM0)
- `baud_rate`: Serial baud rate (default: 115200)
- `control_frequency`: Update rate Hz (default: 50)

---

**gait_controller**
- Generates gait patterns
- Computes foot trajectories
- Handles gait transitions

**Subscribed Topics:**
- `/spider/cmd_vel` (geometry_msgs/Twist)
- `/spider/gait_select` (std_msgs/String)

**Published Topics:**
- `/spider/foot_targets` (custom_msgs/FootTargets)
- `/spider/gait_state` (std_msgs/String)

**Parameters:**
- `gait_type`: Initial gait (default: walk)
- `step_height`: Step height mm (default: 30.0)
- `stride_length`: Stride length mm (default: 50.0)

---

**kinematics_node**
- Solves inverse kinematics
- Converts foot positions to joint angles
- Validates workspace limits

**Subscribed Topics:**
- `/spider/foot_targets` (custom_msgs/FootTargets)

**Published Topics:**
- `/spider/joint_commands` (sensor_msgs/JointState)

**Parameters:**
- `coxa_length`: Hip link length (default: 30.0)
- `femur_length`: Thigh link length (default: 60.0)
- `tibia_length`: Lower leg length (default: 80.0)

---

### spider_cam
Camera streaming and image processing.

**Nodes:**

**cam_relay**
- Connects to ESP32-CAM HTTP stream
- Publishes ROS2 Image messages
- Handles reconnection

**Published Topics:**
- `/spider/camera` (sensor_msgs/Image)
- `/spider/camera_info` (sensor_msgs/CameraInfo)

**Parameters:**
- `camera_url`: ESP32-CAM stream URL (default: http://192.168.4.1:81/stream)
- `frame_rate`: Target FPS (default: 10)

---

**image_processor**
- Optional image processing
- Object detection
- Line following

---

## Arduino Libraries

### SpiderKinematics
Inverse kinematics library for Arduino.

**Usage:**
```cpp
#include <SpiderKinematics.h>

SpiderKinematics ik(30, 60, 80);  // Link lengths

void setup() {
  // Initialize
}

void loop() {
  Vector3 target = {100, 0, -80};  // Target foot position
  LegAngles angles;
  
  if (ik.solve(target, angles)) {
    // Use angles.hip, angles.thigh, angles.knee
  }
}
```

---

### GaitGenerator
Gait pattern generation library.

**Usage:**
```cpp
#include <GaitGenerator.h>

GaitGenerator gait;

void setup() {
  gait.setGait(GAIT_WALK);
  gait.setSpeed(1.0);
}

void loop() {
  gait.update();
  
  for (int i = 0; i < 4; i++) {
    Vector3 footPos = gait.getFootPosition(i);
    // Use footPos for IK
  }
}
```

---

### ServoController
PCA9685 servo control wrapper.

**Usage:**
```cpp
#include <ServoController.h>

ServoController servos;

void setup() {
  servos.begin();
  servos.setFrequency(50);  // 50Hz for servos
}

void loop() {
  servos.setAngle(0, 90);  // Set servo 0 to 90°
  servos.update();
}
```

---

## Building ROS2 Packages

### Prerequisites
```bash
# Install ROS2 Humble
sudo apt install ros-humble-desktop

# Install dependencies
sudo apt install python3-colcon-common-extensions
```

### Build
```bash
# Navigate to workspace
cd ~/spider_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select spider_control

# Source workspace
source install/setup.bash
```

### Run
```bash
# Launch full system
ros2 launch spider_robot bringup.launch.py

# Run individual nodes
ros2 run spider_control servo_bridge
ros2 run spider_control gait_controller
ros2 run spider_control kinematics_node
ros2 run spider_cam cam_relay
```

---

## Installing Arduino Libraries

### Method 1: Arduino IDE

1. Open Arduino IDE
2. Sketch → Include Library → Add .ZIP Library
3. Select library folder
4. Restart Arduino IDE

### Method 2: Manual
```bash
# Copy to Arduino libraries folder
cp -r src/arduino_libraries/SpiderKinematics ~/Arduino/libraries/
cp -r src/arduino_libraries/GaitGenerator ~/Arduino/libraries/
cp -r src/arduino_libraries/ServoController ~/Arduino/libraries/
```

---

## Package Development

### Creating New ROS2 Package
```bash
cd ~/spider_ws/src
ros2 pkg create --build-type ament_python spider_new_package
```

### Package Structure
```
spider_control/
├── package.xml          # Package metadata
├── setup.py            # Python package setup
├── setup.cfg           # Setup configuration
├── spider_control/     # Python package
│   ├── __init__.py
│   └── node.py        # Node implementation
└── test/              # Unit tests
```

### Example Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SpiderNode(Node):
    def __init__(self):
        super().__init__('spider_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from spider robot'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpiderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Custom Messages

### Creating Message Package
```bash
cd ~/spider_ws/src
ros2 pkg create --build-type ament_cmake spider_msgs
```

### Message Definition

**spider_msgs/msg/FootTargets.msg:**
```
std_msgs/Header header
geometry_msgs/Point lf_foot
geometry_msgs/Point rf_foot
geometry_msgs/Point lh_foot
geometry_msgs/Point rh_foot
```

### Build Messages
```bash
colcon build --packages-select spider_msgs
```

---

## Testing

### Unit Tests

**Python (pytest):**
```python
# test/test_kinematics.py
import pytest
from spider_control.kinematics_node import inverse_kinematics

def test_ik_reachable():
    angles = inverse_kinematics(100, 0, -80)
    assert angles is not None
    assert 0 <= angles[0] <= 180

def test_ik_unreachable():
    with pytest.raises(ValueError):
        inverse_kinematics(500, 0, -80)
```

**Run tests:**
```bash
colcon test --packages-select spider_control
colcon test-result --all
```

---

### Integration Tests
```bash
# Launch system
ros2 launch spider_robot bringup.launch.py

# In another terminal, check topics
ros2 topic list
ros2 topic echo /spider/joint_states
```

---

## Troubleshooting

### Build Errors
```bash
# Clean build
rm -rf build install log
colcon build --symlink-install

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Import Errors
```bash
# Source workspace
source install/setup.bash

# Check package installed
ros2 pkg list | grep spider
```

### Serial Connection Issues
```bash
# Check permissions
sudo chmod 666 /dev/ttyACM0

# Add user to dialout group
sudo usermod -aG dialout $USER
```

---

## Documentation

### Generate API Docs

**Python:**
```bash
cd ~/spider_ws/src/spider_control
pdoc --html spider_control
```

**C++ (Doxygen):**
```bash
doxygen Doxyfile
```

---

## Contributing

### Code Style

**Python (PEP 8):**
```bash
# Format code
black spider_control/

# Check style
flake8 spider_control/
```

**C++ (Google Style):**
```bash
clang-format -i *.cpp *.h
```

### Commit Messages
```
feat: Add new gait pattern
fix: Correct IK singularity handling
docs: Update servo_bridge documentation
test: Add kinematics unit tests
```

---

## Performance Optimization

**Python:**
- Use numpy for calculations
- Avoid loops, use vectorization
- Profile with `cProfile`

**Arduino:**
- Minimize serial communication
- Use integer math where possible
- Optimize servo update rate

---

## Notes

- ROS2 packages require proper sourcing of workspace
- Arduino libraries must be in Arduino/libraries folder
- Test individual components before full integration
- Use simulation for initial development
- Always test with robot suspended first
