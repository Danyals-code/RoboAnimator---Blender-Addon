# ops_ui.py — N-panel UI for True RoboAnimator

import bpy
from bpy.types import Panel

def _maybe_prop(col, P, name, **kw):
    """Safely draw a property if it exists."""
    if hasattr(P, name):
        try:
            col.prop(P, name, **kw)
        except Exception:
            pass

def _section(layout, P, prop_flag, title):
    """
    Collapsible section if the boolean flag exists; otherwise a plain box.
    Returns a layout.box() to draw into, or None if collapsed.
    """
    # no flag? show an always-open box
    if not hasattr(P, prop_flag):
        box = layout.box()
        header = box.row()
        header.label(text=title)
        return box

    is_open = getattr(P, prop_flag)
    box = layout.box()
    header = box.row()
    icon = 'TRIA_DOWN' if is_open else 'TRIA_RIGHT'
    # toggle button without emboss to emulate foldout; we still show the label
    header.prop(P, prop_flag, text="", icon=icon, emboss=False)
    header.label(text=title)
    return box if is_open else None


class SG_PT_Panel(Panel):
    bl_label = "True RoboAnimator"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"

    def draw(self, context):
        layout = self.layout
        P = getattr(context.scene, "sg_props", None)
        if P is None:
            col = layout.column(align=True)
            col.label(text="Missing scene props.")
            col.label(text="Register SG_Props and set Scene.sg_props.")
            return

        layout.use_property_split = True
        layout.use_property_decorate = False

        # ---------------------- Instructions ----------------------
        box = _section(layout, P, "show_instructions", "Instructions")
        if box:
            c = box.column(align=True)
            c.label(text="1) Animate chassis (location + rotation).")
            c.label(text="2) Validate Motion (no sideways slip).")
            c.label(text="3) Autocorrect: S-Curve or Linear.")
            c.label(text="4) Pick Speed Profile.")
            c.label(text="5) Build Cache → Attach Drivers (or Bake).")
            c.label(text="6) Export (keyframes / CSV).")
            c.label(text="Tip: If wheels roll backwards, toggle Wheel Forward Invert or swap L/R.")

        # ---------------------- Object Selection ----------------------
        box = _section(layout, P, "show_selection", "Object Selection")
        if box:
            col = box.column(align=True)
            _maybe_prop(col, P, "chassis")
            _maybe_prop(col, P, "right_collection")
            _maybe_prop(col, P, "left_collection")
            _maybe_prop(col, P, "swap_lr")

        # ---------------------- Calibration & Wheel Setup ----------------------
        box = _section(layout, P, "show_calibration", "Calibration & Wheel Setup")
        if box:
            col = box.column(align=True)
            col.label(text="Geometry (meters)")
            _maybe_prop(col, P, "track_width")
            _maybe_prop(col, P, "tire_spacing")
            _maybe_prop(col, P, "auto_radius")
            # Only show wheel_radius if auto detection is off
            if getattr(P, "auto_radius", False) is False:
                _maybe_prop(col, P, "wheel_radius")

            col.separator()
            col.label(text="Wheel Rotation")
            _maybe_prop(col, P, "wheel_axis")
            _maybe_prop(col, P, "rotation_mode")
            _maybe_prop(col, P, "sign_r")
            _maybe_prop(col, P, "sign_l")
            _maybe_prop(col, P, "wheel_forward_invert")

        # ---------------------- Feasibility & Autocorrect ----------------------
        box = _section(layout, P, "show_feasibility", "Feasibility & Autocorrect")
        if box:
            col = box.column(align=True)
            _maybe_prop(col, P, "body_forward_axis")
            _maybe_prop(col, P, "side_tol")
            _maybe_prop(col, P, "autocorrect_mode")
            _maybe_prop(col, P, "bezier_tangent_scale")
            _maybe_prop(col, P, "linear_rotation_fraction")
            _maybe_prop(col, P, "speed_profile")

            # per-profile controls with enable guards
            rowC = col.row(align=True)
            if getattr(P, "speed_profile", "") == 'CONSTANT':
                rowC.enabled = True
            else:
                rowC.enabled = False
            _maybe_prop(rowC, P, "constant_ramp_frames")

            rowG = col.row(align=True)
            rowG.enabled = getattr(P, "speed_profile", "") == 'GLOBAL_EASE'
            _maybe_prop(rowG, P, "timeline_ease_frames")

            rowS = col.row(align=True)
            rowS.enabled = getattr(P, "speed_profile", "") == 'PER_KEY_EASE'
            _maybe_prop(rowS, P, "segment_ease_frames")

            # Validate / Autocorrect buttons
            r = box.row(align=True)
            r.operator("segway.validate_motion", icon='INFO')
            r.operator("segway.autocorrect_bake", icon='MOD_CURVE')
            box.operator("segway.revert_autocorrect", icon='LOOP_BACK')

        # ---------------------- RPM Calculation & Limits ----------------------
        box = _section(layout, P, "show_rpm_calc", "RPM Calculation & Limits")
        if box:
            lim = box.box().column(align=True)
            _maybe_prop(lim, P, "max_rpm")
            _maybe_prop(lim, P, "max_ang_accel_rpm_s")

            r = box.row(align=True)
            r.operator("segway.build_cache", icon='CHECKMARK')
            r.operator("segway.attach_drivers", icon='DRIVER')

            r2 = box.row(align=True)
            r2.operator("segway.bake_wheels", icon='REC')
            r2.operator("segway.clear", icon='X')

        # ---------------------- CSV Engineering Export ----------------------
        box = _section(layout, P, "show_csv_export", "CSV Engineering Export")
        if box:
            col = box.column(align=True)
            _maybe_prop(col, P, "csv_path")
            _maybe_prop(col, P, "sample_mode")
            if getattr(P, "sample_mode", "") == 'FIXED':
                _maybe_prop(col, P, "fixed_rate")
            _maybe_prop(col, P, "angle_unit")
            _maybe_prop(col, P, "angrate_unit")
            _maybe_prop(col, P, "length_unit")
            col.operator("segway.export_csv", icon='EXPORT')

        # ---------------------- Keyframe Export ----------------------
        box = _section(layout, P, "show_anim_export", "Animation Data Export")
        if box:
            col = box.column(align=True)
            _maybe_prop(col, P, "other_export_path")
            _maybe_prop(col, P, "other_export_format")
            _maybe_prop(col, P, "other_angle_unit")
            col.operator("segway.export_keyframes", icon='EXPORT')


# ---------------------- Registration ----------------------
classes = (SG_PT_Panel,)

def register():
    from bpy.utils import register_class
    for c in classes:
        register_class(c)

def unregister():
    from bpy.utils import unregister_class
    for c in reversed(classes):
        unregister_class(c)
