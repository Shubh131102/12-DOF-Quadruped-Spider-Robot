# CAD Files

3D models and mechanical design files for the 12-DOF quadruped spider robot.

## Structure
```
cad/
├── assembly/
│   ├── full_assembly.step
│   ├── full_assembly.stl
│   └── assembly_instructions.pdf
├── body/
│   ├── main_chassis.stl
│   ├── electronics_mount.stl
│   └── battery_holder.stl
├── legs/
│   ├── coxa.stl
│   ├── femur.stl
│   ├── tibia.stl
│   └── foot.stl
├── servo_mounts/
│   ├── servo_bracket.stl
│   └── servo_horn_adapter.stl
└── source_files/
    ├── fusion360/
    ├── solidworks/
    └── step/
```

## Parts Overview

### Body Components

**main_chassis.stl**
- Central body platform
- Mounting points for 4 legs
- Electronics compartment
- Dimensions: 120mm x 80mm x 20mm
- Weight: ~45g (printed in PLA)

**electronics_mount.stl**
- Mounting plate for Arduino and PCA9685
- Cable management channels
- Standoffs for PCB mounting

**battery_holder.stl**
- Secure battery compartment
- Access panel for battery swap
- Fits 7.4V LiPo battery

---

### Leg Components (4 sets required)

Each leg consists of 3 segments with 3 DOF:

**coxa.stl (Hip joint)**
- Connects to body
- Houses first servo (vertical rotation)
- Length: 30mm
- Weight: ~8g

**femur.stl (Upper leg)**
- Second servo mount (forward/backward)
- Length: 60mm
- Weight: ~12g

**tibia.stl (Lower leg)**
- Third servo mount (up/down)
- Length: 80mm
- Foot attachment point
- Weight: ~10g

**foot.stl**
- Rubber grip surface recommended
- Replaceable design
- Weight: ~3g

---

### Servo Mounts

**servo_bracket.stl**
- Universal MG90S servo mount
- 4x M2 screw holes
- Quantity needed: 12

**servo_horn_adapter.stl**
- Connects servo horn to leg segments
- Reinforced connection
- Quantity needed: 12

---

## 3D Printing Guidelines

### Recommended Settings

**Material:** PLA or PETG
- PLA: Easier to print, adequate strength
- PETG: Better impact resistance, slightly heavier

**Layer Height:** 0.2mm (0.15mm for smaller parts)

**Infill:** 
- Body parts: 40% gyroid
- Leg parts: 30% grid
- Servo mounts: 50% honeycomb

**Wall Thickness:** 3 perimeters minimum

**Supports:** Required for:
- Servo mount overhangs
- Body electronics compartment
- Leg joint angles

**Print Orientation:**
- Legs: Print vertically for strength along load axis
- Body: Print flat for dimensional accuracy
- Servo mounts: Orient for minimal supports

---

### Print Time Estimates

| Part | Quantity | Time Each | Total Time |
|------|----------|-----------|------------|
| Main Chassis | 1 | 6 hours | 6 hours |
| Coxa | 4 | 1 hour | 4 hours |
| Femur | 4 | 2 hours | 8 hours |
| Tibia | 4 | 2.5 hours | 10 hours |
| Foot | 4 | 30 min | 2 hours |
| Servo Brackets | 12 | 20 min | 4 hours |
| Electronics Mount | 1 | 2 hours | 2 hours |
| Battery Holder | 1 | 1.5 hours | 1.5 hours |

**Total Print Time:** ~37.5 hours

---

## Assembly Instructions

### Step 1: Prepare Parts
1. Remove supports from all printed parts
2. Sand mating surfaces for smooth fit
3. Test fit servos in brackets
4. Clean screw holes with appropriate tap

### Step 2: Servo Installation
1. Install servo into bracket
2. Secure with M2 x 8mm screws
3. Attach servo horn adapter
4. Repeat for all 12 servos

### Step 3: Leg Assembly
For each leg:
1. Attach coxa to body mounting point
2. Connect coxa servo to femur
3. Connect femur servo to tibia
4. Attach foot to tibia end
5. Route servo cables through body

### Step 4: Electronics Integration
1. Mount Arduino on electronics plate
2. Install PCA9685 PWM driver
3. Mount ESP32-CAM (if using vision)
4. Install power distribution board
5. Connect all servo cables to PCA9685

### Step 5: Final Assembly
1. Secure battery in holder
2. Connect power to BEC
3. Route all cables cleanly
4. Secure electronics mount to chassis
5. Perform cable management

---

## Dimensions and Specifications

**Overall Dimensions:**
- Length: 250mm (body to foot)
- Width: 250mm (leg to leg)
- Height: 120mm (standing)
- Weight: ~850g (complete assembly)

**Leg Specifications:**
- Total leg length: 170mm (coxa + femur + tibia)
- Workspace radius: ~150mm
- Ground clearance: 40-80mm (adjustable)

**Degrees of Freedom:**
- Per leg: 3 DOF (hip yaw, hip pitch, knee pitch)
- Total: 12 DOF
- Servo range: 0-180° (typically use 60-120° range)

---

## Kinematic Chain

Each leg follows this kinematic structure:
```
Body → Coxa (servo 1, yaw) → Femur (servo 2, pitch) → Tibia (servo 3, pitch) → Foot
```

**Joint Angles:**
- Joint 1 (Coxa): ±45° from center
- Joint 2 (Femur): 30-150° (0° = straight down)
- Joint 3 (Tibia): 30-150° (0° = straight)

---

## Material Requirements

**Filament:**
- PLA: ~400g total
- PETG alternative: ~420g total

**Hardware:**
- M2 x 8mm screws: 48 (servo mounting)
- M3 x 10mm screws: 20 (body assembly)
- M3 x 20mm screws: 8 (leg attachment)
- M3 nuts: 28
- Heat-set inserts M3: 16 (optional, for reinforcement)

---

## Modifications and Customization

### Leg Length Adjustment
Modify `femur.stl` and `tibia.stl` length for different stride:
- Shorter legs: Better stability, less reach
- Longer legs: Greater speed, less stable

### Camera Mount
Add to front of chassis:
- ESP32-CAM mounting bracket
- Adjustable tilt mechanism
- Cable routing channel

### Sensor Integration
Mounting points for:
- Ultrasonic sensors (HC-SR04)
- IMU sensor (MPU6050)
- Battery voltage monitor

---

## File Formats

**STL Files:**
- Ready for 3D printing
- No modifications required
- Oriented for optimal printing

**STEP Files:**
- Parametric source files
- Editable in CAD software
- Full assembly with mates

**Source Files:**
- Fusion 360 (.f3d) - Primary design tool
- SolidWorks (.sldprt) - Alternate format
- Native formats for full editing capability

---

## Design Iterations

**Version 1.0:** Initial prototype
- Basic leg design
- Fixed electronics mount

**Version 2.0:** Current design
- Reinforced leg joints
- Modular servo mounts
- Improved cable routing
- Battery access panel

**Future Improvements:**
- [ ] Carbon fiber reinforced legs
- [ ] Modular foot design (different terrains)
- [ ] Integrated IMU mount
- [ ] Wireless charging dock compatibility

---

## Opening CAD Files

### Fusion 360
```bash
# Import STEP file
File → Open → Select assembly.step
# Or open native .f3d file
```

### SolidWorks
```bash
# Import STEP
File → Open → assembly.step
# Set units to mm
```

### FreeCAD (Open Source)
```bash
# Install FreeCAD
sudo apt install freecad

# Open STEP file
freecad assembly.step
```

---

## Exporting for Manufacturing

### For 3D Printing
- Export as STL
- Check normals (outward facing)
- Verify scale (mm)

### For CNC Machining
- Export as STEP or DXF
- Include tolerances in drawing
- Specify material and finish

---

## CAD Tips

**Design Principles:**
1. Print orientation matters for strength
2. Avoid thin walls (<2mm)
3. Add fillets to stress points
4. Design for easy assembly
5. Consider cable routing early

**Common Issues:**
- Servo fit too tight: Scale mount by 101%
- Legs too weak: Increase infill or wall count
- Poor surface finish: Reduce layer height
- Warping: Use brim or raft

---

## Notes

- All dimensions in millimeters
- Designed for MG90S servos (23mm x 12.5mm x 29mm)
- CAD files compatible with most slicers
- Test print one leg assembly before printing all parts
- Consider printing in different colors for easier assembly
