# motion_check.py — properties + motion validation / autocorrect ops

import bpy
from bpy.types import Operator, PropertyGroup
from math import pi, atan2
from .utils import (
    _restore_chassis_keys, _backup_chassis_keys, _edge_ease_progress,
    yaw_to_heading, heading_to_yaw, _body_basis_from_yaw,
    _bezier_point_xy, _bezier_tangent_xy, _build_arc_lut_norm_total_xy, _arc_to_t_from_lut, _trapezoid_s
)

# import the builders defined in Path_check.py
from .Path_check import build_s_ease_curve_and_bake, build_linear_path_and_bake

def _get_wheel_radius(self):
    try:
        return 0.5 * (float(self.left_radius) + float(self.right_radius))
    except Exception:
        return float(getattr(self, "left_radius", 0.04))

def _set_wheel_radius(self, v):
    try:
        v = float(v)
    except Exception:
        return
    self.left_radius = v
    self.right_radius = v


# ---------------------- Scene Properties ----------------------

class SG_Props(bpy.types.PropertyGroup):
    # ---------- Selection ----------
    chassis: bpy.props.PointerProperty(
        name="Chassis",
        description="The main body object of the robot",
        type=bpy.types.Object
    )
    right_wheel: bpy.props.PointerProperty(
        name="Right Wheel",
        type=bpy.types.Object
    )
    left_wheel: bpy.props.PointerProperty(
        name="Left Wheel",
        type=bpy.types.Object
    )

    # NEW: explicit 4-wheel pointers
    wheel_fl: bpy.props.PointerProperty(name="Front-Left Wheel", type=bpy.types.Object)
    wheel_fr: bpy.props.PointerProperty(name="Front-Right Wheel", type=bpy.types.Object)
    wheel_rl: bpy.props.PointerProperty(name="Rear-Left Wheel", type=bpy.types.Object)
    wheel_rr: bpy.props.PointerProperty(name="Rear-Right Wheel", type=bpy.types.Object)

    # Existing: collections for multi-wheel setups (kept for compatibility)
    right_collection: bpy.props.PointerProperty(
        name="Right Wheels (Collection)",
        type=bpy.types.Collection
    )
    left_collection: bpy.props.PointerProperty(
        name="Left Wheels (Collection)",
        type=bpy.types.Collection
    )

    swap_lr: bpy.props.BoolProperty(
        name="Swap L/R Sides",
        default=False
    )

    # ---------- Geometry / wheels ----------
    track_width: bpy.props.FloatProperty(
        name="Track Width (m)",
        default=0.25, min=1e-5, precision=5
    )
    tire_spacing: bpy.props.FloatProperty(
        name="Distance Between Tires (m)",
        default=0.10, min=0.0, precision=5
    )
    right_radius: bpy.props.FloatProperty(
        name="Right Tire Radius (m)",
        default=0.04, min=1e-6, precision=6
    )
    left_radius: bpy.props.FloatProperty(
        name="Left Tire Radius (m)",
        default=0.04, min=1e-6, precision=6
    )
    wheel_radius: bpy.props.FloatProperty(
        name="Wheel Radius (m)",
        description="Alias that writes both Left and Right radii",
        min=1e-6, precision=6,
        get=_get_wheel_radius, set=_set_wheel_radius,
    )

    auto_radius: bpy.props.BoolProperty(
        name="Auto-detect Wheel Radius",
        default=True
    )
    sign_r: bpy.props.EnumProperty(
        name="Right Wheel Direction",
        items=[('PLUS','Forward (+1)',''),('MINUS','Inverted (-1)','')],
        default='PLUS'
    )
    sign_l: bpy.props.EnumProperty(
        name="Left Wheel Direction",
        items=[('PLUS','Forward (+1)',''),('MINUS','Inverted (-1)','')],
        default='PLUS'
    )
    wheel_forward_invert: bpy.props.BoolProperty(
        name="Invert Wheel Forward",
        description="Flip wheel rolling direction relative to body forward (use if the model's front faces the opposite axis)",
        default=False,
    )
    # motion_check.py (inside SG_Props)
    show_import: bpy.props.BoolProperty(
        name="Import Animation",
        description="Show/Hide the Import Animation section",
        default=True,
    )
    # Wheel spin axis (used by kinematics_rpm._AXIS_INDEX[...] lookup)
    wheel_axis: bpy.props.EnumProperty(
        name="Wheel Spin Axis",
        items=[
            ('+X', 'Local +X', ''), ('-X', 'Local -X', ''),
            ('+Y', 'Local +Y', ''), ('-Y', 'Local -Y', ''),
            ('+Z', 'Local +Z', ''), ('-Z', 'Local -Z', ''),
        ],
        default='+X',
    )

    # ---------- Feasibility / Autocorrect (path geometry) ----------
    body_forward_axis: bpy.props.EnumProperty(
        name="Body Forward Axis",
        items=[('+X','Local +X',''), ('-X','Local -X',''), ('+Y','Local +Y',''), ('-Y','Local -Y','')],
        default='+Y'
    )
    side_tol: bpy.props.FloatProperty(
        name="Side-slip Tolerance (m/s)",
        description="Allowed lateral velocity. Larger values tolerate more sideways motion.",
        default=0.01, min=0.0, precision=6
    )
    turn_tol: bpy.props.FloatProperty(
        name="Turning Curvature Limit (1/m)",
        description="Max allowed curvature. 0 = unlimited.",
        default=0.0, min=0.0, precision=6
    )
    linear_rotation_fraction: bpy.props.FloatProperty(
        name="Linear Mode: Rotation Fraction",
        description="Rotate-in / rotate-out share per span in Linear mode",
        default=0.25, min=0.0, max=0.45, precision=3
    )
    bezier_tangent_scale: bpy.props.FloatProperty(
        name="S-Ease Tangent Scale",
        description="Controls Bezier tangent length relative to chord",
        default=0.25, min=0.01, max=1.0, precision=3
    )
    speed_profile: bpy.props.EnumProperty(
        name="Speed Profile",
        items=[
            ('CONSTANT','Constant','Global trapezoid over total length'),
            ('GLOBAL_EASE','Global Ease','Ease at timeline edges'),
            ('PER_KEY_EASE','Per-Key Ease','Ease in/out inside each segment'),
        ],
        default='CONSTANT'
    )
    segment_ease_frames: bpy.props.IntProperty(
        name="Per-Segment Ease Frames",
        default=6, min=0, max=1000
    )
    timeline_ease_frames: bpy.props.IntProperty(
        name="Timeline Ease Frames",
        default=15, min=0, max=5000
    )
    constant_ramp_frames: bpy.props.IntProperty(
        name="Constant: Ramp Frames",
        description="Ramp-in/out frames for trapezoid speed (only in CONSTANT)",
        default=0, min=0, max=100000
    )
    feasibility_rate: bpy.props.IntProperty(
        name="Feasibility Sample Rate (Hz)",
        default=60, min=1, max=10000
    )
    autocorrect_mode: bpy.props.EnumProperty(
        name="Autocorrect Mode",
        items=[('SEASE','Smooth Curve (S-Ease)',''), ('LINEAR','Linear (Rotate–Move–Rotate)','')],
        default='SEASE'
    )

    # ---------- Export / engineering ----------
    fixed_rate: bpy.props.BoolProperty(
        name="Export at Fixed Rate",
        default=False
    )
    fixed_rate_hz: bpy.props.IntProperty(
        name="Rate (Hz)",
        default=60, min=1, max=5000
    )
    angle_unit: bpy.props.EnumProperty(
        name="Angle Unit",
        items=[('RAD','radians',''),('DEG','degrees','')],
        default='RAD'
    )
    angrate_unit: bpy.props.EnumProperty(
        name="Angular Rate",
        items=[('RPM','rpm',''),('RPS','rps',''),('DEGS','deg/s','')],
        default='RPM'
    )
    length_unit: bpy.props.EnumProperty(
        name="Length Unit",
        items=[('M','meters',''),('CM','centimeters','')],
        default='M'
    )

    # ---------- Safety limits ----------
    max_rpm: bpy.props.FloatProperty(
        name="Max Wheel Speed (RPM)",
        description="Hard limit on per-frame wheel speed. 0 = disabled.",
        min=0.0, soft_max=100000.0, default=0.0,
    )
    max_ang_accel_rpm_s: bpy.props.FloatProperty(
        name="Max Wheel Accel (RPM/s)",
        description="Hard limit on per-frame wheel angular acceleration. 0 = disabled.",
        min=0.0, soft_max=1_000_000.0, default=0.0,
    )

    # ---------- UI foldouts ----------
    show_instructions: bpy.props.BoolProperty(name="Show Instructions", default=False)
    show_selection:   bpy.props.BoolProperty(name="Show Object Selection", default=True)
    show_calibration: bpy.props.BoolProperty(name="Show Calibration", default=False)
    show_feasibility: bpy.props.BoolProperty(name="Show Feasibility", default=True)
    show_rpm_calc:    bpy.props.BoolProperty(name="Show RPM Calculation", default=False)
    show_anim_export: bpy.props.BoolProperty(name="Show Animation Data Export", default=False)
    show_csv_export:  bpy.props.BoolProperty(name="Show CSV Engineering Export", default=False)

    # ---------- Keyframe export ----------
    other_export_path: bpy.props.StringProperty(
        name="Anim Data File",
        default="//anim_keyframes.csv",
        subtype='FILE_PATH'
    )
    csv_export_path: bpy.props.StringProperty(
        name="Engineering CSV",
        default="//wheel_rpm.csv",
        subtype='FILE_PATH'
    )

# ---------------------- Operators ----------------------

class SG_OT_AutocorrectBake(Operator):
    bl_idname = "segway.autocorrect_bake"
    bl_label = "Autocorrect & Bake"
    bl_description = "Bake using the selected Autocorrect Mode (S-Ease or Linear)"

    def execute(self, context):
        P = context.scene.sg_props
        mode = P.autocorrect_mode
        try:
            if mode == 'SEASE':
                n = build_s_ease_curve_and_bake(context)
            elif mode == 'LINEAR':
                n = build_linear_path_and_bake(context)
            else:
                self.report({'ERROR'}, "Set Autocorrect Mode to S-Ease or Linear.")
                return {'CANCELLED'}
        except Exception as e:
            self.report({'ERROR'}, f"Autocorrect failed: {e}")
            return {'CANCELLED'}
        self.report({'INFO'}, f"Autocorrect baked {n} frames. Re-run Validate Motion.")
        return {'FINISHED'}


class SG_OT_AutocorrectSEase(Operator):
    bl_idname = "segway.autocorrect_sease"
    bl_label = "Autocorrect & Bake (Smooth S-Ease)"

    def execute(self, context):
        P = context.scene.sg_props
        if P.autocorrect_mode != 'SEASE':
            self.report({'ERROR'}, "Set Autocorrect Mode to 'Smooth Curve (S-Ease)'.")
            return {'CANCELLED'}
        try:
            n = build_s_ease_curve_and_bake(context)
        except Exception as e:
            self.report({'ERROR'}, f"Autocorrect failed: {e}")
            return {'CANCELLED'}
        self.report({'INFO'}, f"Autocorrect baked {n} frames. Re-run Validate Motion.")
        return {'FINISHED'}


class SG_OT_AutocorrectLinear(Operator):
    bl_idname = "segway.autocorrect_linear"
    bl_label = "Autocorrect & Bake (Linear: Rotate–Move–Rotate)"

    def execute(self, context):
        P = context.scene.sg_props
        if P.autocorrect_mode != 'LINEAR':
            self.report({'ERROR'}, "Set Autocorrect Mode to 'Linear (Rotate–Move–Rotate)'.")
            return {'CANCELLED'}
        try:
            n = build_linear_path_and_bake(context)
        except Exception as e:
            self.report({'ERROR'}, f"Autocorrect failed: {e}")
            return {'CANCELLED'}
        self.report({'INFO'}, f"Linear autocorrect baked {n} frames. Re-run Validate Motion.")
        return {'FINISHED'}


class SG_OT_RevertAutocorrect(Operator):
    bl_idname = "segway.revert_autocorrect"
    bl_label = "Revert Autocorrect"

    def execute(self, context):
        ch = context.scene.sg_props.chassis
        if not ch:
            self.report({'ERROR'}, "Assign the Chassis.")
            return {'CANCELLED'}
        ok = _restore_chassis_keys(ch)
        if not ok:
            self.report({'ERROR'}, "No backup found to restore.")
            return {'CANCELLED'}
        self.report({'INFO'}, "Original chassis keyframes restored.")
        return {'FINISHED'}


# ---------------------- Registration ----------------------
# Only register the operators here. SG_Props is registered in __init__.py
classes = (
    SG_OT_AutocorrectBake,
    SG_OT_AutocorrectSEase,
    SG_OT_AutocorrectLinear,
    SG_OT_RevertAutocorrect,
)

def register():
    from bpy.utils import register_class
    for c in classes:
        register_class(c)

def unregister():
    from bpy.utils import unregister_class
    for c in reversed(classes):
        unregister_class(c)
