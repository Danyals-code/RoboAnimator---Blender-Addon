# ops_ui.py — N-panel UI for True RoboAnimator (native panels + subpanels)
import bpy
from bpy.types import Panel

# ---------- helpers ----------
def _P(context):
    return getattr(context.scene, "sg_props", None)

def _maybe_prop(col, P, name, **kw):
    if hasattr(P, name):
        try:
            col.prop(P, name, **kw)
        except Exception:
            pass

# ---------- MAIN PANEL (collapsible) ----------
class SG_PT_Main(Panel):
    bl_label = "True RoboAnimator"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"
    # Start closed so the whole block is collapsible like native Blender sections.
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        # Intentionally minimal: subpanels contain all content.
        if _P(context) is None:
            self.layout.label(text="SG_Props not registered. Enable the add-on.", icon='ERROR')

# ---------- SUBPANELS ----------
class SG_PT_Instructions(Panel):
    bl_label = "Instructions"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"
    bl_parent_id = "SG_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        col = self.layout.column(align=True)
        col.label(text="1) Animate chassis (location + rotation).")
        col.label(text="2) Validate Motion (no sideways slip).")
        col.label(text="3) Autocorrect: S-Curve or Linear.")
        col.label(text="4) Pick Speed Profile.")
        col.label(text="5) Build Cache → Attach Drivers (or Bake).")
        col.label(text="6) Export (Raw / Engineering CSV).")
        col.label(text="Tip: If wheels roll backwards, toggle Wheel Forward Invert or swap L/R.")

class SG_PT_SelectionCalib(Panel):
    bl_label = "Object Selection & Calibration"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"
    bl_parent_id = "SG_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        P = _P(context);
        if not P: return
        col = self.layout.column(align=True)

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

class SG_PT_Feasibility(Panel):
    bl_label = "Feasibility & Autocorrect"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"
    bl_parent_id = "SG_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        P = _P(context);
        if not P: return
        col = self.layout.column(align=True)

        _maybe_prop(col, P, "body_forward_axis")
        _maybe_prop(col, P, "side_tol")
        _maybe_prop(col, P, "autocorrect_mode")

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

class SG_PT_RPM(Panel):
    bl_label = "RPM Calculation & Limits"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"
    bl_parent_id = "SG_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        P = _P(context);
        if not P: return
        col = self.layout.column(align=True)

        lim = col.box().column(align=True)
        _maybe_prop(lim, P, "max_rpm")
        _maybe_prop(lim, P, "max_ang_accel_rpm_s")

        r = col.row(align=True)
        r.operator("segway.build_cache", icon='CHECKMARK')
        r.operator("segway.attach_drivers", icon='DRIVER')

        r2 = col.row(align=True)
        r2.operator("segway.bake_wheels", icon='REC')
        r2.operator("segway.clear", icon='X')

# ---------- EXPORT PANEL + SUBPANELS ----------
class SG_PT_Export(Panel):
    bl_label = "Export CSV"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"
    bl_parent_id = "SG_PT_Main"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        # Header only; content is in subpanels.
        pass

class SG_PT_Export_Raw(Panel):
    bl_label = "Raw animation data"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"
    bl_parent_id = "SG_PT_Export"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        P = _P(context);
        if not P: return
        col = self.layout.column(align=True)
        _maybe_prop(col, P, "other_export_path")
        _maybe_prop(col, P, "other_export_format")  # CSV or JSON
        _maybe_prop(col, P, "other_angle_unit")     # RAD / DEG
        col.operator("segway.export_keyframes", icon='EXPORT')

class SG_PT_Export_Eng(Panel):
    bl_label = "Engineering data"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"
    bl_parent_id = "SG_PT_Export"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        P = _P(context)
        if not P: return

        layout = self.layout
        col = layout.column(align=True)
        col.scale_x = 0.95

        box = col.box()
        inner = box.column(align=True)

        _maybe_prop(inner, P, "csv_path")
        _maybe_prop(inner, P, "sample_mode")
        if getattr(P, "sample_mode", "") == 'FIXED':
            _maybe_prop(inner, P, "fixed_rate")
        _maybe_prop(inner, P, "angle_unit")
        _maybe_prop(inner, P, "angrate_unit")
        _maybe_prop(inner, P, "length_unit")

        inner.operator("segway.export_csv", icon='EXPORT')

# ---------- registration ----------
classes = (
    SG_PT_Main,
    SG_PT_Instructions,
    SG_PT_SelectionCalib,
    SG_PT_Feasibility,
    SG_PT_RPM,
    SG_PT_Export,
    SG_PT_Export_Raw,
    SG_PT_Export_Eng,
)

def register():
    from bpy.utils import register_class
    for c in classes:
        register_class(c)

def unregister():
    from bpy.utils import unregister_class
    for c in reversed(classes):
        unregister_class(c)
