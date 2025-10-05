# motion_check.py — properties + motion validation / autocorrect ops

import bpy
from bpy.types import Operator, PropertyGroup
from .utils import _restore_chassis_keys

from math import pi

# ---------------------- Scene Properties ----------------------
# --- REPLACE your current SG_Props with this full version ---

class SG_Props(bpy.types.PropertyGroup):
    # ---------- Selection ----------
    chassis: bpy.props.PointerProperty(
        name="Chassis (animated)",
        type=bpy.types.Object
    )

    # NEW: Explicit single-wheel object hooks (preferred by importer)
    wheel_left: bpy.props.PointerProperty(
        name="Left Wheel",
        type=bpy.types.Object,
        description="Left wheel object for rotation animation"
    )
    wheel_right: bpy.props.PointerProperty(
        name="Right Wheel",
        type=bpy.types.Object,
        description="Right wheel object for rotation animation"
    )

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
        default=0.40, min=0.0, precision=5
    )
    auto_radius: bpy.props.BoolProperty(
        name="Auto-detect Wheel Radius",
        default=True
    )
    wheel_radius: bpy.props.FloatProperty(
        name="Wheel Radius (m)",
        default=0.06, min=1e-5, precision=5
    )

    # Existing axis (kept for back-compat)
    wheel_axis: bpy.props.EnumProperty(
        name="Wheel Rotation Axis",
        items=[('X','X',''),('Y','Y',''),('Z','Z','')],
        default='X'
    )

    # NEW: Explicit spin axis (used by importer first, falls back to wheel_axis)
    wheel_spin_axis: bpy.props.EnumProperty(
        name="Wheel Spin Axis",
        description="Local axis around which wheels spin (used by importer)",
        items=[('X','X-axis',''),('Y','Y-axis',''),('Z','Z-axis','')],
        default='Y'
    )

    rotation_mode: bpy.props.EnumProperty(
        name="Rotation Mode",
        items=[('EULER','Euler',''),('QUAT','Quaternion','')],
        default='EULER'
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

    # ---------- Feasibility / Autocorrect (path geometry) ----------
    body_forward_axis: bpy.props.EnumProperty(
        name="Body Forward Axis",
        items=[('+X','Local +X',''), ('-X','Local -X',''), ('+Y','Local +Y',''), ('-Y','Local -Y','')],
        default='+Y'
    )
    side_tol: bpy.props.FloatProperty(
        name="Sideways Tolerance (m/s)",
        default=0.02, min=0.0, precision=6
    )

    autocorrect_mode: bpy.props.EnumProperty(
        name="Autocorrect Mode (Path Geometry)",
        items=[
            ('SEASE','Smooth Curve (S-Ease)','Smooth Bezier segments with curvature clamp'),
            ('LINEAR','Linear (Rotate–Move–Rotate)','Rotate to face, move straight, rotate to final'),
        ],
        default='SEASE',
    )
    bezier_tangent_scale: bpy.props.FloatProperty(
        name="Tangent Scale (S-Ease)",
        description="Handle length as fraction of pose distance (larger = rounder arcs). Curvature is clamped by track_width/2.",
        default=0.35, min=0.05, max=2.0, precision=3
    )
    linear_rotation_fraction: bpy.props.FloatProperty(
        name="Rotation Fraction (Linear)",
        description="Per segment, fraction of frames used for the initial and final rotations (each).",
        default=0.25, min=0.0, max=0.45, precision=3
    )

    # ---------- Speed profiles (timing) ----------
    speed_profile: bpy.props.EnumProperty(
        name="Speed Profile (Timing)",
        items=[
            ('CONSTANT',     "Constant (Uniform+Ramps)",    "Constant speed with ramps (linear accel/decel)"),
            ('GLOBAL_EASE',  "Ease — Whole Timeline",       "Slow-in/fast/slow-out across the entire timeline"),
            ('PER_KEY_EASE', "Ease — Per Keyframe Segment", "Each keyframe-to-keyframe segment eases (single slider)"),
        ],
        default='CONSTANT',
    )
    constant_ramp_frames: bpy.props.IntProperty(
        name="Ramp Frames (Constant)",
        description="Frames to ramp at start and end (linear accel → cruise → linear decel).",
        min=0, max=2000, default=12,
    )
    timeline_ease_frames: bpy.props.IntProperty(
        name="Ease Frames (Timeline)",
        description="Frames to ramp at the start and end of the whole timeline (GLOBAL_EASE only)",
        min=0, max=400, default=15,
    )
    segment_ease_frames: bpy.props.IntProperty(
        name="Ease Frames (Per Segment)",
        description="Symmetric ease-in/out within each keyframe segment.",
        min=0, max=400, default=6,
    )

    # ---------- CSV / units ----------
    csv_path: bpy.props.StringProperty(
        name="CSV File",
        default="//robot_anim.csv",
        subtype='FILE_PATH'
    )
    sample_mode: bpy.props.EnumProperty(
        name="Sampling",
        items=[('FRAME','Every Frame (scene FPS)',''), ('FIXED','Fixed Rate (Hz)','')],
        default='FRAME'
    )
    fixed_rate: bpy.props.IntProperty(name="Rate (Hz)", min=1, max=5000, default=100)
    angle_unit: bpy.props.EnumProperty(name="Angle Unit", items=[('RAD','radians',''),('DEG','degrees','')], default='RAD')
    angrate_unit: bpy.props.EnumProperty(name="Angular Rate", items=[('RPM','rpm',''),('RPS','rps',''),('DEGS','deg/s','')], default='RPM')
    length_unit: bpy.props.EnumProperty(name="Length Unit", items=[('M','meters',''),('CM','centimeters','')], default='M')

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
    other_export_format: bpy.props.EnumProperty(
        name="Format",
        items=[('CSV','CSV',''),('JSON','JSON','')],
        default='CSV'
    )
    other_angle_unit: bpy.props.EnumProperty(
        name="Angle Unit (Anim)",
        items=[('RAD','radians',''),('DEG','degrees','')],
        default='RAD'
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
