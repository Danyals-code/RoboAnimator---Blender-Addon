# ops_ui.py — UI for True RoboAnimator (revised for wheels + calibrated import)
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

def _pick_flag(P, candidates, fallback_name=None, fallback_default=True):
    for n in candidates:
        if hasattr(P, n):
            return n, bool(getattr(P, n))
    if fallback_name and hasattr(P, fallback_name):
        return fallback_name, bool(getattr(P, fallback_name))
    return None, fallback_default

# ---------- main panel ----------
class SG_PT_Panel(Panel):
    bl_label = "True RoboAnimator"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"

    def _section(self, layout, prop_flag: str, title: str):
        box = layout.box()
        header = box.row()
        P = bpy.context.scene.sg_props
        is_open = bool(getattr(P, prop_flag, False))
        icon = 'TRIA_DOWN' if is_open else 'TRIA_RIGHT'
        header.prop(P, prop_flag, text="", icon=icon, emboss=False)
        header.label(text=title)
        if not is_open:
            return None
        # inner margin
        margin = box.column(align=False)
        margin.separator(factor=0.3)
        inner = margin.column(align=True)
        inner.use_property_split = True
        inner.use_property_decorate = False
        return inner

    def draw(self, context):
        layout = self.layout
        P = _P(context)
        if not P:
            layout.label(text="SG_Props not registered.", icon='ERROR')
            return

        layout.use_property_split = True
        layout.use_property_decorate = False

        # ---------------- Instructions ----------------
        col = self._section(layout, "show_instructions", "Instructions")
        if col:
            c = col.column(align=True)
            c.label(text="1) Assign Chassis + Wheels in Calibration.")
            c.label(text="2) Import CSV → animates chassis + tires.")
            c.label(text="3) Validate Motion, then Autocorrect.")
            c.label(text="4) Choose Speed Profile and bake/drive.")
            c.label(text="5) Export (Raw / Engineering CSV).")

        # ---------------- Selection & Calibration ----------------
        col = self._section(layout, "show_selection", "Object Selection & Calibration")
        if col:
            # Selection
            _maybe_prop(col, P, "chassis")
            # system selector
            _maybe_prop(col, P, "wheel_system")

            # 2W vs 4W UI
            if getattr(P, "wheel_system", "2W") == "2W":
                _maybe_prop(col, P, "wheel_left")
                _maybe_prop(col, P, "wheel_right")
            else:
                row = col.row(align=True);
                row.label(text="Left")
                _maybe_prop(col, P, "wheel_fl");
                _maybe_prop(col, P, "wheel_rl")
                row = col.row(align=True);
                row.label(text="Right")
                _maybe_prop(col, P, "wheel_fr");
                _maybe_prop(col, P, "wheel_rr")

            # keep legacy bits visible but clearly “legacy/fallback”
            col.separator()
            col.label(text="Legacy (collections, optional):", icon='INFO')
            _maybe_prop(col, P, "left_collection")
            _maybe_prop(col, P, "right_collection")

            col.separator(factor=2.0)  # spacing

            _maybe_prop(col, P, "track_width")
            _maybe_prop(col, P, "tire_spacing")
            _maybe_prop(col, P, "auto_radius")
            if not getattr(P, "auto_radius", True):
                _maybe_prop(col, P, "wheel_radius")

            col.separator(factor=2.0)

            _maybe_prop(col, P, "wheel_spin_axis")
            _maybe_prop(col, P, "rotation_mode")
            _maybe_prop(col, P, "sign_r")
            _maybe_prop(col, P, "sign_l")
            _maybe_prop(col, P, "wheel_forward_invert")

        # ---------------- Import Animation ----------------
        col = self._section(layout, "show_import", "Import Animation")
        if col:
            chassis_ok = bool(getattr(P, "chassis", None))

            r = col.row(align=True)
            r.enabled = chassis_ok
            op = r.operator("segway.import_csv_anim", text="Import CSV Animation", icon='IMPORT')
            # Force using calibrated object so user doesn't need to select anything
            try:
                op.use_calibrated_object = True
            except Exception:
                pass

            if not chassis_ok:
                col.label(text="Set ‘Chassis’ in Calibration first.", icon='ERROR')

            # If the active object has an attached CSV, show its Text name for provenance
            obj = context.object
            if obj and isinstance(getattr(obj, "get", None), type(dict.get)) and 'roboanim_csv_text' in obj:
                col.separator()
                col.label(text=f"Attached CSV: {obj['roboanim_csv_text']}", icon='TEXT')

        # ---------------- Feasibility & Autocorrect ----------------
        col = self._section(layout, "show_feasibility", "Feasibility & Autocorrect")
        if col:
            # Autocorrect settings
            _maybe_prop(col, P, "body_forward_axis")
            _maybe_prop(col, P, "side_tol")
            _maybe_prop(col, P, "autocorrect_mode")

            row_se = col.row(align=True)
            row_se.enabled = getattr(P, "autocorrect_mode", "") == 'SEASE'
            _maybe_prop(row_se, P, "bezier_tangent_scale")

            row_lin = col.row(align=True)
            row_lin.enabled = getattr(P, "autocorrect_mode", "") == 'LINEAR'
            _maybe_prop(row_lin, P, "linear_rotation_fraction")

            col.separator(factor=2.0)  # spacing

            # Speed profile
            _maybe_prop(col, P, "speed_profile")

            row_const = col.row(align=True)
            row_const.enabled = getattr(P, "speed_profile", "") == 'CONSTANT'
            _maybe_prop(row_const, P, "constant_ramp_frames")

            row_tl = col.row(align=True)
            row_tl.enabled = getattr(P, "speed_profile", "") == 'GLOBAL_EASE'
            _maybe_prop(row_tl, P, "timeline_ease_frames")

            row_seg = col.row(align=True)
            row_seg.enabled = getattr(P, "speed_profile", "") == 'PER_KEY_EASE'
            _maybe_prop(row_seg, P, "segment_ease_frames")

            col.separator(factor=2.0)

            # Operators
            r = col.row(align=True)
            r.operator("segway.validate_motion", icon='INFO')
            r.operator("segway.autocorrect_bake", icon='MOD_CURVE')
            col.operator("segway.revert_autocorrect", icon='LOOP_BACK')

        # ---------------- RPM Calculation & Limits ----------------
        col = self._section(layout, "show_rpm_calc", "RPM Calculation & Limits")
        if col:
            _maybe_prop(col, P, "max_rpm")
            _maybe_prop(col, P, "max_ang_accel_rpm_s")
            col.separator()
            r1 = col.row(align=True)
            r1.operator("segway.build_cache", icon='CHECKMARK')
            r1.operator("segway.attach_drivers", icon='DRIVER')
            r2 = col.row(align=True)
            r2.operator("segway.bake_wheels", icon='REC')
            r2.operator("segway.clear", icon='X')

        # ---------------- Export (collapsible, two parts spaced) ----------------
        flag_name, is_open = _pick_flag(
            P,
            candidates=["show_export"],
            fallback_name=("show_csv_export" if hasattr(P, "show_csv_export") else
                           ("show_anim_export" if hasattr(P, "show_anim_export") else None)),
            fallback_default=True
        )

        exp_box = layout.box()
        header = exp_box.row()
        icon = 'TRIA_DOWN' if is_open else 'TRIA_RIGHT'
        if flag_name:
            header.prop(P, flag_name, text="", icon=icon, emboss=False)
        else:
            header.label(text="", icon=icon)
        header.label(text="Export")

        if is_open:
            margin = exp_box.column(align=False)
            margin.separator(factor=0.3)
            col = margin.column(align=True)
            col.use_property_split = True
            col.use_property_decorate = False

            # ---- Raw Animation Data ----
            _colA = col.column(align=True)
            _maybe_prop(_colA, P, "other_export_path")
            _maybe_prop(_colA, P, "other_export_format")
            _maybe_prop(_colA, P, "other_angle_unit")
            _colA.separator(factor=0.7)
            _colA.operator("segway.export_keyframes", text="Export Raw Animation Data", icon='EXPORT')

            col.separator(factor=1.7)

            # ---- Engineering CSV Data ----
            _colB = col.column(align=True)
            _maybe_prop(_colB, P, "csv_path")
            _maybe_prop(_colB, P, "sample_mode")
            if getattr(P, "sample_mode", "") == 'FIXED':
                _maybe_prop(_colB, P, "fixed_rate")
            _maybe_prop(_colB, P, "angle_unit")
            _maybe_prop(_colB, P, "angrate_unit")
            _maybe_prop(_colB, P, "length_unit")
            _colB.separator(factor=0.7)
            _colB.operator("segway.export_csv", text="Export Engineering Requirements", icon='EXPORT')

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
