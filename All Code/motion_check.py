# motion_check.py — properties + motion validation / autocorrect ops

import bpy
from bpy.types import Operator, PropertyGroup
from .utils import _restore_chassis_keys, _driver_key, _AXIS_INDEX
from math import pi

# ---------------------- Scene Properties ----------------------
class SG_Props(PropertyGroup):
    # Selection
    chassis: bpy.props.PointerProperty(name="Chassis (animated)", type=bpy.types.Object)
    right_collection: bpy.props.PointerProperty(name="Right Wheels (Collection)", type=bpy.types.Collection)
    left_collection: bpy.props.PointerProperty(name="Left Wheels (Collection)", type=bpy.types.Collection)
    swap_lr: bpy.props.BoolProperty(name="Swap L/R Sides", default=False)

    # Geometry / wheels
    track_width: bpy.props.FloatProperty(name="Track Width (m)", default=0.25, min=1e-5, precision=5)
    tire_spacing: bpy.props.FloatProperty(name="Distance Between Tires (m)", default=0.40, min=0.0, precision=5)
    auto_radius: bpy.props.BoolProperty(name="Auto-detect Wheel Radius", default=True)
    wheel_radius: bpy.props.FloatProperty(name="Wheel Radius (m)", default=0.06, min=1e-5, precision=5)
    wheel_axis: bpy.props.EnumProperty(name="Wheel Rotation Axis",
        items=[('X','X',''),('Y','Y',''),('Z','Z','')], default='X')
    rotation_mode: bpy.props.EnumProperty(name="Rotation Mode",
        items=[('EULER','Euler',''),('QUAT','Quaternion','')], default='EULER')
    sign_r: bpy.props.EnumProperty(name="Right Wheel Direction",
        items=[('PLUS','Forward (+1)',''),('MINUS','Inverted (-1)','')], default='PLUS')
    sign_l: bpy.props.EnumProperty(name="Left Wheel Direction",
        items=[('PLUS','Forward (+1)',''),('MINUS','Inverted (-1)','')], default='PLUS')

    wheel_forward_invert: bpy.props.BoolProperty(
        name="Invert Wheel Forward",
        description="Flip wheel rolling direction relative to body forward",
        default=False,
    )

    # Feasibility / Autocorrect
    body_forward_axis: bpy.props.EnumProperty(
        name="Body Forward Axis",
        items=[('+X','Local +X',''), ('-X','Local -X',''),
               ('+Y','Local +Y',''), ('-Y','Local -Y','')],
        default='+X'
    )
    autocorrect_mode: bpy.props.EnumProperty(
        name="Autocorrect Mode",
        items=[('SEASE','Smooth Curve (S-Ease)',''),
               ('LINEAR','Linear (Rotate–Move–Rotate)','')],
        default='SEASE'
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
classes = (
    SG_Props,
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
