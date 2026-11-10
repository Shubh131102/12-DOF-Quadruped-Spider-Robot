# 12-DOF Quadruped Robot

A four-legged walking robot designed, 3D printed, wired, and programmed from scratch at UC Riverside. Three joints per leg, servo-driven tripod gait, and FEA-optimized legs that walk on uneven terrain.

<p align="center">
  <img src="media/WhatsApp%20Image%202025-10-01%20at%2014.58.55_7e579c70.jpg" alt="12-DOF quadruped robot with orange 3D-printed legs" width="420">
</p>

## Results

| | |
|---|---|
| Stiffness-to-weight gain from FEA on the legs | **25%** |
| Actuators | 12 servos, 3 per leg |
| Walking speed | About 8.5 cm/s (0.8 s step cycle) |
| Stride length | About 6.8 cm |
| Power draw | About 7.5 W with 6 servos under load |
| Outcome | Stable walking on uneven terrain |

## What I designed

- **Chassis and leg linkages** in SolidWorks, printed in PLA over several iterations
- **FEA-driven legs:** used ANSYS to remove material from low-stress regions and reinforce high-stress areas
- **Actuator sizing:** picked the 12 servos from joint torque requirements based on leg geometry and robot weight
- **Hip adduction mechanism** for side-to-side balance while walking
- **Harness routing** for 12 servos across all joints

## What broke, and how I fixed it

| Problem | Root cause | Fix |
|---|---|---|
| Arduino kept resetting mid-gait | Voltage sag when several servos moved at once | Upgraded power wiring gauge and separated logic and servo power |
| Servos overheating | Heat trapped inside the chassis | Redesigned the chassis geometry for airflow |
| Intermittent servo dropouts | Cable fatigue at the moving joints | Rerouted the harness to reduce bending at the joints |

## What's in this repo

| Path | Contents |
|---|---|
| `python_gui/app.py` | Tkinter control app: individual servo control, preset poses, gait selection, emergency stop, serial link to the Arduino |
| `launch/bringup.launch.py` | ROS 2 launch file for camera relay, servo control, and teleop |
| `config/servos.yaml` | Servo channel and limit configuration |
| `bom/bom.csv` | Bill of materials |
| `docs/` | Project presentation |
| `media/` | Photos of the built robot |

## Hardware

Arduino UNO R4 WiFi, PCA9685 PWM driver, 12 MG996R / SG90 servos, ESP32-CAM for live video, battery with BEC.

## Run the control app

```bash
pip install pyserial
python python_gui/app.py
```

Select the Arduino's serial port in the app, connect, then pick a pose or gait.

## Team

[Shubham Jangle](https://github.com/Shubh131102), Shubham Dehrgave, Soumya Mondal

## License

MIT
