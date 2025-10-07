<p align="center">
  <img src="addon%20thumbnail.png" width="640" alt="True RoboAnimator preview">
</p>


# True RoboAnimator

**Version:** 1.0.0  
**Author:** Danyal S.  
**Blender Support:** 4.2+

---

## Overview
**True RoboAnimator** is a Blender add-on that generates **engineering-accurate animations for differential-drive robots**.  
It provides full motion validation, curvature-safe path smoothing, speed-profile control, and automatic wheel-rotation drivers.  
Designed for robotics engineers and animators who need physically consistent, non-slip motion for simulation or presentation.

---

## Key Features
- **Nonholonomic Path Validation** — checks chassis motion for sideways slip violations.  
- **Autocorrect Modes:**
  - *S-Ease (Smooth Curve)* — builds Bezier segments with curvature clamps.
  - *Linear (Rotate–Move–Rotate)* — rotates to face target, drives straight, rotates to final heading.
- **Speed Profiles:**
  - *Constant (Trapezoid)* — uniform speed with configurable acceleration/deceleration ramps.
  - *Global Ease* — smooth start/end for the entire timeline.
  - *Per-Key Ease* — segment-level easing between keyframes.
- **Wheel Kinematics Cache** — computes RPM, angular acceleration, and rotation angles per frame.
- **Automatic Wheel Drivers** — attach rotation drivers to wheel meshes or empties.
- **CSV Import/Export:**
  - Export robot trajectories with time, pose, and wheel motion.
  - Import external CSV animation data into Blender.
- **Safety Limits:** user-definable wheel speed (RPM) and angular acceleration (RPM/s) caps.

---

## Installation
1. Download the repository or `.zip` of the add-on.  
2. In Blender:  
   - `Edit` → `Preferences` → `Add-ons` → `Install`.  
   - Select the downloaded `.zip`.  
3. Enable **True RoboAnimator** in the Add-on list.  
4. Access it from the **3D Viewport → N-Panel → True RoboAnimator** tab.

---

## Basic Workflow
1. **Assign Objects**
   - Set your chassis object and left/right wheel collections.
   - Adjust track width and wheel radius (or enable Auto-detect).

2. **Animate Chassis**
   - Keyframe chassis motion (position + rotation).  
   - Use *Autocorrect & Bake* to generate feasible differential motion.

3. **Validate**
   - Run *Validate Motion* to confirm no sideways slip or limit violations.

4. **Build Cache**
   - Generates wheel angular motion data and stores it in Blender’s driver namespace.

5. **Attach Drivers**
   - Automatically applies rotation drivers to wheel meshes.

6. **Export Data (optional)**
   - Export to CSV for further analysis or robot playback.

---

## Requirements
- **Blender 4.2.0 or newer**  
- Python 3.11+ (bundled with Blender)  
- No external dependencies required.

---

## Known Limitations
- Designed for **differential-drive** robots only.  
- Requires valid keyframes on chassis before baking.  
- Scene FPS affects motion integration accuracy.

---

## License
**GNU General Public License v3.0 (GPL-3.0)**  
This add-on follows Blender’s licensing requirements.  
You are free to use, modify, and redistribute it under the same license.

---

## Citation
If used in academic work, please cite as:

> Sarfraz, D. (2025). *True RoboAnimator: Engineering-Accurate Differential-Drive Animation Toolkit for Blender 4.2*.  
> Graduate Thesis Project, Ulsan National Institute of Science and Technology (UNIST).

---

## Contact
For issues or feature requests, open a GitHub issue or contact the author.
- [LinkedIn — Danyal Sarfraz](https://www.linkedin.com/in/designbydanyal/)
- [Lab Website — Integration and Innovation Design Lab, UNIST](https://iidl.unist.ac.kr/)
