import bpy
from .selection import auto_radius, refresh_wheels
from .feasibility import analyze, autocorrect, revert_autocorrect
from .rpm import build

class RefreshWheels(bpy.types.Operator):
    bl_idname = "robo.refresh_wheels"
    bl_label = "Refresh Wheels"
    bl_description = "Scan Left/Right collections and report wheel counts"
    def execute(self, ctx):
        info = refresh_wheels(ctx)
        msg = "L {} | R {} | Total {} / Expected {}".format(
            info['L'], info['R'], info['total'], info['want']
        )
        if info['mismatch']:
            self.report({'WARNING'}, "Count mismatch: " + msg)
        else:
            self.report({'INFO'}, "Counts ok: " + msg)
        return {'FINISHED'}

class AutoRadius(bpy.types.Operator):
    bl_idname = "robo.auto_radius"
    bl_label = "Auto-Detect Radius"
    bl_description = "Set wheel radius from a wheel object's dimensions"
    def execute(self, ctx):
        try:
            r = auto_radius(ctx)
        except Exception as e:
            self.report({'ERROR'}, str(e)); return {'CANCELLED'}
        self.report({'INFO'}, "Wheel radius = {:.4f} m".format(r))
        return {'FINISHED'}

# legacy path validator now calls feasibility.analyze
class ValidatePath(bpy.types.Operator):
    bl_idname = "robo.validate_path"
    bl_label = "Validate Path"
    bl_description = "Basic check of chassis animation (feasibility)"
    def execute(self, ctx):
        res = analyze(ctx)
        level = 'INFO' if res.get("ok") else 'WARNING'
        self.report({level}, res.get("msg", "Done"))
        return {'FINISHED'}

class BuildCache(bpy.types.Operator):
    bl_idname = "robo.build_cache"
    bl_label = "Build Cache"
    bl_description = "Compute wheel angles and store cache"
    def execute(self, ctx):
        build(ctx)
        self.report({'INFO'}, "Cache built.")
        return {'FINISHED'}

class AttachDrivers(bpy.types.Operator):
    bl_idname = "robo.attach_drivers"
    bl_label = "Attach Drivers"
    bl_description = "Attach rotation drivers to wheels on selected axis"
    def execute(self, ctx):
        self.report({'INFO'}, "Drivers: implement after cache math.")
        return {'FINISHED'}

# feasibility buttons
class ValidateMotion(bpy.types.Operator):
    bl_idname = "robo.validate_motion"
    bl_label = "Validate Motion"
    bl_description = "Check sideways drift vs tolerance using chosen forward axis"
    def execute(self, ctx):
        res = analyze(ctx)
        level = 'INFO' if res.get("ok") else 'WARNING'
        self.report({level}, res.get("msg", ""))
        return {'FINISHED'}

class AutoCorrectPath(bpy.types.Operator):
    bl_idname = "robo.autocorrect_path"
    bl_label = "Autocorrect"
    bl_description = "Plan path autocorrection (S-curve or Bezier)"
    def execute(self, ctx):
        res = autocorrect(ctx)
        self.report({'INFO'}, res.get("msg", "Autocorrect set"))
        return {'FINISHED'}

class RevertAutoCorrect(bpy.types.Operator):
    bl_idname = "robo.revert_autocorrect"
    bl_label = "Revert Autocorrect"
    bl_description = "Clear planned autocorrection"
    def execute(self, ctx):
        res = revert_autocorrect(ctx)
        self.report({'INFO'}, res.get("msg", "Cleared"))
        return {'FINISHED'}
