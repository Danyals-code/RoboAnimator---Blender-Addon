<p align="center">
  <img src="addon%20thumbnail.png" width="640" alt="True RoboAnimator preview">
</p>


# True RoboAnimator

**Version:** 1.4.1
**Author:** Danyal S.
**Blender Support:** 4.2 LTS through 5.0


## Overview
True RoboAnimator is a Blender add-on that generates engineering-accurate animations for wheeled robots. It supports differential-drive, differential + caster, and skid-steer platforms with 2, 3, 4, or 6 wheels, and provides motion validation, curvature-safe path smoothing, speed-profile control, and automatic wheel-rotation drivers. It is aimed at robotics engineers and technical animators who need physically consistent, non-slip motion for simulation or presentation.


## Supported Wheel Configurations
Pick the layout that matches your robot; the panel then shows exactly the wheel slots you need.

| Config | Wheels                       | Kinematics                                                     |
|--------|------------------------------|----------------------------------------------------------------|
| 2      | Left 01, Right 01            | Standard differential drive.                                   |
| 3      | Left 01, Right 01, Caster    | Differential drive + a passive caster that rolls off chassis center forward motion only. |
| 4      | Left 01/02, Right 01/02      | Skid-steer. Both left wheels share one angle; both right wheels share one angle. |
| 6      | Left 01/02/03, Right 01/02/03| Rocker / rover skid-steer, same per-side sharing as the 4-wheel case. |

Per-side wheel angles follow the classic differential-drive integration:

    dsL = dx - 0.5 * track_width * dpsi
    dsR = dx + 0.5 * track_width * dpsi
    dth = (ds / wheel_radius) * side_sign

The caster (3-wheel only) uses `ds_caster = dx`, independent of yaw, because a real caster pivots freely and its wheel spin tracks forward travel of the chassis center.


## Key Features
- **Nonholonomic path validation.** Checks chassis motion for sideways slip violations against a configurable tolerance.
- **Autocorrect modes.**
  - *S-Ease (Smooth Curve)*: Bezier segments with a curvature clamp derived from the track width.
  - *Linear (Rotate, Move, Rotate)*: rotates in place to face the target, drives straight, then rotates to the final heading.
- **Speed profiles.**
  - *Constant (Trapezoid)*: uniform cruise speed with configurable linear accel/decel ramp frames.
  - *Global Ease*: smooth start and end applied across the whole timeline.
  - *Per-Key Ease*: symmetric ease-in/out inside each keyframe-to-keyframe segment.
- **Per-configuration wheel kinematics.** Cache holds thetaL, thetaR (and thetaC on 3-wheel) per frame, plus RPM and angular acceleration diagnostics.
- **Automatic wheel drivers.** Attach rotation drivers to every configured wheel, in Euler or Quaternion mode, with each wheel's own rest orientation.
- **CSV import and export.**
  - Export trajectories with time, pose, wheel angles, and wheel rates. thetaC / rateC columns are added when the 3-wheel config is active.
  - Import external CSV animation data back into Blender using scene FPS and start frame.
- **Safety limits.** User-definable caps on driven-wheel speed (RPM) and angular acceleration (RPM/s). The caster is diagnostic only.


## Installation
1. Download the latest `.zip` release, or clone this repository and zip the contents.
2. In Blender go to *Edit > Preferences > Get Extensions > Install from Disk* (Blender 4.2+).
3. Select the zip and enable **True RoboAnimator** in the list.
4. Open the sidebar in the 3D Viewport (`N`) and switch to the **True RoboAnimator** tab.

Because drivers run Python expressions, first-time users on a fresh scene should ensure *Auto Run Python Scripts* is enabled (or trust the file) so the wheel drivers evaluate correctly.


## Panel Layout
The N-panel is organized into numbered stages so the workflow reads top to bottom:

1. **Rig Setup.** Assign the chassis, pick a wheel count (2 / 3 / 4 / 6), then fill in the visible wheel slots.
2. **Calibration.** Track width, tire spacing, wheel radius, wheel rotation axis and mode, per-side rolling direction.
3. **Motion Path.** Body forward axis, sideways slip tolerance, autocorrect mode, speed profile, and Validate / Autocorrect / Revert actions.
4. **Wheel Drivers.** Safety limits, then Build Cache, Attach Drivers, Bake Wheels, and Clear.
5. **Import CSV.** Load a chassis + wheel trajectory back from CSV using scene FPS and start frame.
6. **Export Keyframes.** Keyframe-only CSV/JSON snapshot at existing chassis keys.
7. **Engineering CSV.** Time-series CSV at scene rate or a fixed Hz, in your preferred units.


## Basic Workflow
1. **Rig up.** Set the chassis object and wheel count. Assign each visible wheel slot.
2. **Calibrate.** Set track width and either enable Auto-detect for the wheel radius or type it in. Pick the wheel rotation axis and mode.
3. **Animate the chassis.** Keyframe location and rotation on the frames that define the path.
4. **Validate.** Run *Validate Motion* to confirm no sideways slip.
5. **Autocorrect if needed.** Pick a mode and speed profile, then *Autocorrect & Bake*. Re-run *Validate Motion*.
6. **Build cache.** Integrates per-frame wheel angles for every configured wheel and stores them in Blender's driver namespace.
7. **Attach drivers or bake wheels.** Drivers for live playback, bake for exporting per-frame keyframes to other tools.
8. **Export.** Engineering CSV or keyframe snapshot for downstream analysis or hardware playback.


## Requirements
- Blender 4.2 LTS or newer (tested through Blender 5.0).
- Python 3.11+ (bundled with Blender).
- No external Python dependencies.


## Known Limitations
- Skid-steer 4/6-wheel configs assume both wheels on a side share the same angle. Independently steered wheels are not modelled.
- Requires at least two keyed poses on the chassis (location + rotation) before autocorrect and cache steps.
- Scene FPS affects motion-integration accuracy. Set the target FPS before building the cache.


## What's New in 1.4.1
- **Engineering CSV now exports PHYSICAL thetas and rates.** Positive theta / positive rate means the wheel rolls forward, independent of how the wheel model is oriented. The cache still stores visual (local-axis) thetas for Blender drivers; the exporter multiplies by each wheel's stored sign to convert. Previously an L/R rig with opposite axle orientations produced CSV values that looked "swapped" between left and right.
- **Tangent Scale no longer plateaus.** Removed the `5 * rho` cap (max L = 2.5 * track_width) and the iterative curvature-shrink loop from the S-Ease segment builder. Tangent Scale now controls curve handle length directly, capped only at 0.49 * chord distance to keep the curve from folding back.
- **Independent start / end tangents.** The `Tangent Scale` slider is split into `Tangent Scale Start` (P1 handle) and `Tangent Scale End` (P2 handle) so each segment can be shaped asymmetrically. Both default to 0.35.

## What's New in 1.4.0
- **Wheel rotation direction fix.** The auto-sign math cross-producted the wheel axis with `down` instead of `up`, so every wheel was rolling backwards. Now derived correctly from the rolling-without-slip constraint and verified on paper.
- **Track width auto-detect.** Track width is now measured as the world distance between `Left Wheel 01` and `Right Wheel 01` by default. The old `Distance Between Tires` field was unused metadata; removed.
- **`Invert Wheel Rotation` toggle removed.** With the sign math fixed there's no reason to expose it.
- **`Wheel Rotation Axis` auto-set.** Whenever you change Body Forward Axis the wheel axis snaps to the perpendicular horizontal axis. Still overridable.
- **Assigning an object auto-applies scale and rotation.** When you pick a chassis or wheel in the panel, its rotation and scale are baked into the mesh (skipped for parented, shared-mesh, already-animated, or already-identity objects), so `matrix_world`, `dimensions`, and the visualization all reflect the true rig geometry.
- **Panel reorganized.** Viewport Helpers now sits right under Instructions. Import CSV, Export Keyframes, and Engineering CSV collapse into one `Import / Export` section with a mode-tab enum.

## What's New in 1.3.2
- The solution-path overlay is now a **live preview**. Switching between S-Ease and Linear, sliding Tangent Scale, or changing Rotation Fraction updates the cyan path instantly in the viewport, without needing to Autocorrect & Bake first. It samples S-Ease Bezier geometry directly and treats Linear as a polyline through the waypoints, so what you see is exactly what a bake would produce spatially.
- N-panel cleanup: dropped section numbering (Motion Path, Wheel Drivers, etc), removed most nested sub-boxes, trimmed labels, and cut the color legend. Same content, less visual noise.

## What's New in 1.3.1
- Wheel rotation direction is now auto-detected per wheel from the wheel's world-space rotation axis and the body forward vector, so `Left/Right/Caster Wheel Direction` and the `Invert Wheel Forward` toggle are gone. If all wheels roll the wrong way, use the single `Invert Wheel Rotation` fallback.
- Raw / solution path overlays now work on Blender 4.4+ slotted actions: the action-fcurve compat layer walks `layers -> strips -> channelbags` so location fcurves are found regardless of storage.
- Wheel-axis gizmo shortened. Length now scales to about 2 x tire width (using the smallest bbox dimension of the wheel as the tire-width proxy) instead of the previous over-long line.

## What's New in 1.3.0
- New Viewport Helpers overlay drawn as construction lines in every 3D View, updated live as settings change:
  - Green forward-axis arrow on the chassis for the selected Body Forward Axis.
  - Colored rotation-axis line through every configured wheel (red for left, blue for right, yellow for the caster) with a small tick at the positive-theta end.
  - Orange raw path with waypoint dots at every chassis keyframe (falls back to the autocorrect backup when present, so you see the original intent after baking).
  - Cyan solution path that samples the current chassis animation at every frame.
- Per-toggle checkboxes plus a master toggle in a new Viewport Helpers section of the panel.
- Overlay always draws through geometry so it stays visible during modeling.

## What's New in 1.2.0
- New wheel selection UI. Individual wheel Object slots (Left Wheel 01, Right Wheel 02, ...) replace the old Left/Right collection pickers.
- New Wheel Configuration selector: 2, 3, 4, or 6 wheels. The panel shows exactly the slots that config needs.
- Caster wheel support for the 3-wheel config, with its own rest orientation, driver expression, and CSV column.
- Skid-steer 4 and 6-wheel configs replicate the per-side angle to every wheel on that side.
- Panel rewrite: numbered stages, consistent sub-boxes, and icons on every action.

## What's New in 1.1.0
- Blender 5.0 support. Added a compatibility layer for slotted actions introduced in Blender 4.4 so action creation, fcurve reads, and fcurve writes are safe across 4.2, 4.3, 4.4, and 5.0.
- Fixed missing top-level imports that previously prevented the module from loading in some Blender builds.
- CSV import now uses the scene's FPS and start frame instead of a hard-coded 30 FPS / frame-1 origin.
- Fixed `Invert Wheel Forward`: previously the sign flip and an extra `dx = -dx` cancelled each other for translation while inverting only yaw. The flag now consistently inverts every wheel's rotation.
- Fixed cumulative drift in the CSV quaternion importer: rest orientation is captured once per wheel instead of being re-read after each keyframe write.
- Fixed `Bake Wheels` in quaternion mode: each wheel is now baked against its own rest orientation rather than the first wheel's rest.
- Consistent ASCII text in UI labels and README for scripting-friendly output.


## License
GNU General Public License v3.0 (GPL-3.0). This add-on follows Blender's licensing requirements. You are free to use, modify, and redistribute it under the same license.


## Citation
If used in academic work, please cite as:

> Sarfraz, D. (2025). *True RoboAnimator: Engineering-Accurate Wheeled-Robot Animation Toolkit for Blender*.
> Graduate Thesis Project, Ulsan National Institute of Science and Technology (UNIST).


## Contact
For issues or feature requests, open a GitHub issue or contact the author.
- LinkedIn: [Danyal Sarfraz](https://www.linkedin.com/in/danyal-sarfraz)
