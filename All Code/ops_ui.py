# ops_ui.py — N-panel UI for True RoboAnimator
import bpy
from bpy.types import Panel

# ---------- tiny helpers ----------
def _maybe_prop(col, P, name, **kw):
    """Draw a property only if it exists; no crashes while you iterate."""
    if hasattr(P, name):
        try:
            col.prop(P, name, **kw)
        except Exception:
            pass

def _section(layout, P, flag_name: str, title: str):
    """
    Collapsible section controlled by Scene.sg_props.<flag_name>.
    If the flag doesn't exist, render an always-open box.
    """
    box = layout.box()
    header = box.row()
    header.use_property_split = False
    header.use_property_decorate = False

    if hasattr(P, flag_name):
        is_open = bool(getattr(P, flag_name))
        icon = 'TRIA_DOWN' if is_open else 'TRIA_RIGHT'
        header.prop(P, flag_name, text=title, icon=icon, emboss=False)
        return box.column(align=True) if is_open else None

    # Fallback: no flag on SG_Props → always-open box (no dropdown)
    header.label(text=title)
    return box.column(align=True)


# ---------- main panel ----------
class SG_PT_Panel(Panel):
    bl_label = "True RoboAnimator"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"

    def draw(self, context):
        layout = self.layout
        P = getattr(context.scene, "sg_props", None)
        if P is None:
            layout.label(text="SG_Props not registered. Enable the add-on.", icon='ERROR')
            return

        layout.use_property_split = True
        layout.use_property_decorate = False

        # --- Instructions ---
        box = _section(layout, P, "show_instructions", "Instructions")
        if box:
            box.label(text="1) Animate chassis (location + rotation).")
            box.label(text="2) Validate Motion (no sideways slip).")
            box.label(text="3) Autocorrect: S-Curve or Linear.")
            box.label(text="4) Pick Speed Profile.")
            box.label(text="5) Build Cache → Attach Drivers (or Bake).")
            box.label(text="6) Export (keyframes / CSV).")
            box.label(text="Tip: If wheels roll backwards, toggle Wheel Forward Invert or swap L/R.")

        # --- Object Selection ---
        col = _section(layout, P, "show_selection", "Object Selection")
        if col:
            _maybe_prop(col, P, "chassis")
            _maybe_prop(col, P, "right_collection")
            _maybe_prop(col, P, "left_collection")
            _maybe_prop(col, P, "swap_lr")

        # --- Calibration & Wheel Setup ---
        col = _section(layout, P, "show_calibration", "Calibration & Wheel Setup")
        if col:
            col.label(text="Geometry (meters)")
            _maybe_prop(col, P, "track_width")
            _maybe_prop(col, P, "tire_spacing")
            _maybe_prop(col, P, "auto_radius")
            if getattr(P, "auto_radius", False) is False:
                _maybe_prop(col, P, "wheel_radius")

            col.separator()
            col.label(text="Wheel Rotation")
            _maybe_prop(col, P, "wheel_axis")
            _maybe_prop(col, P, "rotation_mode")
            _maybe_prop(col, P, "sign_r")
            _maybe_prop(col, P, "sign_l")
            _maybe_prop(col, P, "wheel_forward_invert")

        # --- Feasibility & Autocorrect ---
        col = _section(layout, P, "show_feasibility", "Feasibility & Autocorrect")
        if col:
            _maybe_prop(col, P, "body_forward_axis")
            _maybe_prop(col, P, "side_tol")
            _maybe_prop(col, P, "autocorrect_mode")

            # Geometry knobs depending on chosen mode
            row_se = col.row(align=True); row_se.enabled = getattr(P, "autocorrect_mode", "") == 'SEASE'
            _maybe_prop(row_se, P, "bezier_tangent_scale")

            row_lin = col.row(align=True); row_lin.enabled = getattr(P, "autocorrect_mode", "") == 'LINEAR'
            _maybe_prop(row_lin, P, "linear_rotation_fraction")

            col.separator()
            _maybe_prop(col, P, "speed_profile")

            row_const = col.row(align=True); row_const.enabled = getattr(P, "speed_profile", "") == 'CONSTANT'
            _maybe_prop(row_const, P, "constant_ramp_frames")

            row_tl = col.row(align=True); row_tl.enabled = getattr(P, "speed_profile", "") == 'GLOBAL_EASE'
            _maybe_prop(row_tl, P, "timeline_ease_frames")

            row_seg = col.row(align=True); row_seg.enabled = getattr(P, "speed_profile", "") == 'PER_KEY_EASE'
            _maybe_prop(row_seg, P, "segment_ease_frames")

            col.separator()
            r = col.row(align=True)
            r.operator("segway.validate_motion", icon='INFO')
            r.operator("segway.autocorrect_bake", icon='MOD_CURVE')
            col.operator("segway.revert_autocorrect", icon='LOOP_BACK')

        # --- RPM Calculation & Limits ---
        col = _section(layout, P, "show_rpm_calc", "RPM Calculation & Limits")
        if col:
            lim = col.box().column(align=True)
            _maybe_prop(lim, P, "max_rpm")
            _maybe_prop(lim, P, "max_ang_accel_rpm_s")

            r = col.row(align=True)
            r.operator("segway.build_cache", icon='CHECKMARK')
            r.operator("segway.attach_drivers", icon='DRIVER')

            r2 = col.row(align=True)
            r2.operator("segway.bake_wheels", icon='REC')
            r2.operator("segway.clear", icon='X')

        # --- Animation Data Export (Keyframes) ---
        col = _section(layout, P, "show_anim_export", "Animation Data Export (Keyframes)")
        if col:
            _maybe_prop(col, P, "other_export_path")
            _maybe_prop(col, P, "other_export_format")
            _maybe_prop(col, P, "other_angle_unit")
            col.operator("segway.export_keyframes", icon='EXPORT')

        # --- CSV Engineering Export ---
        col = _section(layout, P, "show_csv_export", "CSV Engineering Export")
        if col:
            _maybe_prop(col, P, "csv_path")
            _maybe_prop(col, P, "sample_mode")
            if getattr(P, "sample_mode", "") == 'FIXED':
                _maybe_prop(col, P, "fixed_rate")
            _maybe_prop(col, P, "angle_unit")
            _maybe_prop(col, P, "angrate_unit")
            _maybe_prop(col, P, "length_unit")
            col.operator("segway.export_csv", icon='EXPORT')

# ---------- registration ----------
classes = (SG_PT_Panel,)

def register():
    from bpy.utils import register_class
    for c in classes:
        register_class(c)

def unregister():
    from bpy.utils import unregister_class
    for c in reversed(classes):
        unregister_class(c)
