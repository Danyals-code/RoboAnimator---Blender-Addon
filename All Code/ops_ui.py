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
    Collapsible section controlled by a Scene.sg_props boolean like 'show_xxx'.
    If the flag doesn't exist, we render an always-open box.
    """
    box = layout.box()
    header = box.row()
    if hasattr(P, flag_name):
        is_open = getattr(P, flag_name)
        icon = 'TRIA_DOWN' if is_open else 'TRIA_RIGHT'
        header.prop(P, flag_name, text="", icon=icon, emboss=False)
        header.label(text=title)
        return box if is_open else None
    # no flag on props -> just always open
    header.label(text=title)
    return box

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
            c = box.column(align=True)
            c.label(text="1) Animate chassis (location + rotation).")
            c.label(text="2) Validate Motion (no sideways slip).")
            c.label(text="3) Autocorrect: S-Curve or Linear.")
            c.label(text="4) Pick Speed Profile.")
            c.label(text="5) Build Cache → Attach Drivers (or Bake).")
            c.label(text="6) Export (keyframes / CSV).")
            c.label(text="Tip: If wheels roll backwards, toggle Wheel Forward Invert or swap L/R.")

        # --- Object Selection + Calibration (merged) ---
        box = _section(layout, P, "show_selection", "Object Selection & Calibration")
        if box:
            col = box.column(align=True)
            # Selection
            _maybe_prop(col, P, "chassis")
            _maybe_prop(col, P, "right_collection")
            _maybe_prop(col, P, "left_collection")
            _maybe_prop(col, P, "swap_lr")

            col.separator()
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
        box = _section(layout, P, "show_feasibility", "Feasibility & Autocorrect")
        if box:
            col = box.column(align=True)
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

        # --- Animation Data Export (Keyframes) ---
        box = _section(layout, P, "show_anim_export", "Animation Data Export (Keyframes)")
        if box:
            col = box.column(align=True)
            _maybe_prop(col, P, "other_export_path")
            _maybe_prop(col, P, "other_export_format")
            _maybe_prop(col, P, "other_angle_unit")
            col.operator("segway.export_keyframes", icon='EXPORT')

        # --- CSV Engineering Export ---
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
