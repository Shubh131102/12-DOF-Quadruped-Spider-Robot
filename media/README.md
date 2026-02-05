# Media

Demonstration videos, images, and visualizations for the 12-DOF quadruped spider robot.

## Structure
```
media/
├── videos/
│   ├── walking_demo.mp4
│   ├── gait_comparison.mp4
│   ├── assembly_timelapse.mp4
│   └── terrain_test.mp4
├── images/
│   ├── robot_assembled.jpg
│   ├── leg_mechanism.jpg
│   ├── electronics_layout.jpg
│   ├── cad_rendering.png
│   └── kinematics_diagram.png
└── gifs/
    ├── walking_loop.gif
    ├── trot_gait.gif
    └── servo_test.gif
```

## Videos

### walking_demo.mp4
- Robot walking with standard walk gait
- Demonstrates stable 4-legged locomotion
- Shows forward/backward/turning capabilities
- Duration: 1-2 minutes

### gait_comparison.mp4
- Side-by-side comparison of walk, trot, and crawl gaits
- Speed and stability differences
- Terrain adaptability demonstration

### assembly_timelapse.mp4
- Full assembly process from 3D printed parts to working robot
- Wiring and electronics integration
- Calibration and testing
- Duration: 3-5 minutes (time-lapse)

### terrain_test.mp4
- Robot navigating different surfaces
- Flat ground, carpet, obstacles, inclines
- Demonstrates adaptability and stability

## Images

### robot_assembled.jpg
- Fully assembled robot from multiple angles
- Front, side, top, and 3/4 views
- Shows mechanical design and aesthetics

### leg_mechanism.jpg
- Close-up of leg assembly
- 3-DOF joint mechanism
- Servo mounting and linkages

### electronics_layout.jpg
- Internal electronics arrangement
- Arduino, PCA9685, ESP32-CAM placement
- Wiring and cable management
- Battery mounting

### cad_rendering.png
- CAD model render from Fusion 360 or SolidWorks
- Exploded view showing all components
- Assembly relationships

### kinematics_diagram.png
- Inverse kinematics diagram
- Link lengths and joint angles labeled
- Coordinate frames
- Workspace visualization

## GIFs

### walking_loop.gif
- Short looping animation of walking
- ~3 second loop
- Ideal for README and documentation

### trot_gait.gif
- Trot gait pattern demonstration
- Shows diagonal leg pairs moving together
- ~2 second loop

### servo_test.gif
- Individual servo testing sequence
- Shows full range of motion
- Useful for debugging visualization

## Creating Media

### Recording Videos

**Camera Setup:**
- Use smartphone or webcam
- 1080p resolution minimum
- 30 fps
- Good lighting
- Stable tripod/mount

**For Walking Demos:**
```bash
# Position camera at robot eye level
# 2-3 meters distance
# Capture full robot in frame
# Follow robot movement smoothly
```

**Screen Recording (for simulations):**
```bash
# Linux - SimpleScreenRecorder
simplescreenrecorder

# macOS - QuickTime
# File → New Screen Recording

# Windows - OBS Studio
```

---

### Taking Photos

**High-Quality Photos:**
- Use DSLR or good smartphone camera
- Natural lighting or softbox
- Clean background (white/black)
- Multiple angles
- Macro lens for detail shots

**Recommended Angles:**
1. Front view (show face/camera)
2. Side view (show leg mechanism)
3. Top view (show symmetry)
4. 3/4 view (show overall form)
5. Close-ups (servos, joints, electronics)

---

### Creating GIFs

**From Video:**
```bash
ffmpeg -i walking_demo.mp4 -vf "fps=10,scale=640:-1:flags=lanczos" \
  -t 3 walking_loop.gif
```

**Optimize GIF:**
```bash
gifsicle -O3 --colors 128 walking_loop.gif -o walking_loop_optimized.gif
```

**From Image Sequence:**
```bash
convert -delay 10 -loop 0 frame*.png walking.gif
```

---

### Editing Videos

**Trimming:**
```bash
ffmpeg -i input.mp4 -ss 00:00:10 -t 00:00:30 -c copy output.mp4
```

**Compression:**
```bash
ffmpeg -i input.mp4 -vcodec libx264 -crf 23 output.mp4
```

**Add Text Overlay:**
```bash
ffmpeg -i input.mp4 -vf "drawtext=text='Walk Gait':fontsize=24:x=10:y=10" output.mp4
```

---

## Capture Checklist

### For Assembly Video
- [ ] Unpacking 3D printed parts
- [ ] Servo installation
- [ ] Leg assembly (all 4 legs)
- [ ] Electronics mounting
- [ ] Wiring connections
- [ ] First power-on
- [ ] Calibration process
- [ ] First successful walk

### For Gait Demonstration
- [ ] Robot in neutral pose
- [ ] Walk gait (front/side view)
- [ ] Trot gait
- [ ] Crawl gait
- [ ] Turning left/right
- [ ] Speed variations
- [ ] Emergency stop

### For Technical Documentation
- [ ] CAD renders (all views)
- [ ] Kinematic diagrams
- [ ] Wiring schematics
- [ ] PCB layout (if custom)
- [ ] Component labels
- [ ] Dimension annotations

---

## Photography Settings

**For Robot Photos:**
- Aperture: f/5.6 - f/8 (good depth of field)
- ISO: 100-400 (low noise)
- Shutter: 1/60s or faster
- White balance: Daylight or custom
- Focus: Manual on robot center

**For Action Shots:**
- Shutter priority mode
- 1/250s or faster
- Continuous autofocus
- Burst mode for multiple frames

---

## Video Specifications

**Recommended Settings:**
- **Resolution:** 1920x1080 (Full HD)
- **Frame rate:** 30 fps
- **Codec:** H.264
- **Bitrate:** 8-12 Mbps
- **Format:** MP4

**For Slow Motion:**
- Record at 60 fps
- Playback at 30 fps
- Shows smooth servo motion

---

## Image Specifications

**Photos:**
- **Format:** JPEG (compressed) or PNG (lossless)
- **Resolution:** 3000x2000 or higher
- **DPI:** 300 for print quality
- **Color space:** sRGB

**Diagrams:**
- **Format:** PNG or SVG
- **Resolution:** 2000x1500 minimum
- **Background:** Transparent or white
- **Annotations:** Clear labels, arrows

---

## Lighting Setup

**Basic 3-Point Lighting:**

1. **Key Light:** Main light at 45° angle
2. **Fill Light:** Soften shadows opposite key light
3. **Back Light:** Rim light from behind robot

**For Product Photography:**
- Use diffused lighting (softbox)
- Avoid harsh shadows
- White background with reflector
- Multiple light sources

---

## Scene Composition

**For Walking Demos:**
- Clean floor (white/gray/black)
- Minimal distractions in background
- Include reference object for scale
- Show full walking cycle

**For Close-ups:**
- Shallow depth of field
- Focus on specific mechanism
- Include context (what it connects to)
- Show detail without clutter

---

## Organizing Media Files

**Naming Convention:**
```
YYYYMMDD_description_version.ext

Examples:
20240215_walking_demo_v1.mp4
20240215_leg_mechanism_front.jpg
20240216_trot_gait_loop.gif
```

**Folder Structure:**
```
media/
├── raw/           # Unedited source files
├── edited/        # Processed files
└── final/         # Published versions
```

---

## Usage in Documentation

### Embedding in README

**Images:**
```markdown
![Robot Assembly](media/images/robot_assembled.jpg)
```

**GIFs:**
```markdown
![Walking Demo](media/gifs/walking_loop.gif)
```

**Videos:**
```markdown
[Watch Walking Demo](media/videos/walking_demo.mp4)

Or link to YouTube:
[![Walking Demo](thumbnail.jpg)](https://youtube.com/watch?v=...)
```

---

### Creating Thumbnails
```bash
# Extract frame from video
ffmpeg -i walking_demo.mp4 -ss 00:00:05 -frames:v 1 thumbnail.jpg

# Resize for web
convert thumbnail.jpg -resize 640x360 thumbnail_small.jpg
```

---

## Demo Scenarios

### Scenario 1: Basic Walking

**Setup:**
- Flat surface (table or floor)
- Good lighting
- Clear background

**Actions:**
1. Robot in standing pose (5 sec)
2. Walk forward (10 sec)
3. Stop
4. Walk backward (10 sec)
5. Turn left 90° (5 sec)
6. Turn right 90° (5 sec)

---

### Scenario 2: Gait Comparison

**Setup:**
- Same as Scenario 1
- Split-screen or sequential

**Actions:**
1. Walk gait (15 sec)
2. Transition to trot (5 sec)
3. Trot gait (15 sec)
4. Transition to crawl (5 sec)
5. Crawl gait (15 sec)

---

### Scenario 3: Terrain Test

**Setup:**
- Multiple surfaces prepared
- Camera follows robot

**Actions:**
1. Flat floor
2. Carpet/textured surface
3. Small obstacles (books, blocks)
4. Incline (ramp ~10-15°)
5. Recovery from obstacle

---

## Performance Metrics to Capture

**Quantitative:**
- Walking speed (cm/s)
- Step height (mm)
- Stride length (mm)
- Battery life (minutes)
- Control latency (ms)

**Qualitative:**
- Stability (subjective 1-10)
- Smoothness of motion
- Noise level
- Terrain adaptability

---

## Publishing Guidelines

**For Social Media:**
- Short clips (15-30 sec)
- Vertical format for mobile (9:16)
- Add captions/text
- Background music (royalty-free)

**For GitHub README:**
- Compressed GIFs (<5MB)
- Representative images
- Concise video links

**For Technical Documentation:**
- High-resolution images
- Detailed diagrams
- Full-length videos

---

## Copyright and Attribution

- All media created for this project
- Credit collaborators if applicable
- Use royalty-free music for videos
- Include robot branding/watermark

---

## Tools and Software

**Video Editing:**
- DaVinci Resolve (free)
- Adobe Premiere Pro
- iMovie (macOS)

**Photo Editing:**
- GIMP (free)
- Adobe Photoshop
- Lightroom

**Screen Recording:**
- OBS Studio (free, cross-platform)
- SimpleScreenRecorder (Linux)

**GIF Creation:**
- FFmpeg (command-line)
- GIMP
- Online tools (ezgif.com)

---

## Notes

- Capture B-roll (extra footage) for editing flexibility
- Keep raw files backed up
- Export multiple resolutions for different platforms
- Test media playback on target devices
- Compress large files before committing to Git (use Git LFS if needed)
