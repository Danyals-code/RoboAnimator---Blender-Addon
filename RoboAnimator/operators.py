import bpy
from .selection import auto_radius, refresh_wheels

class RefreshWheels(bpy.types.Operator):
    bl_idname = "robo.refresh_wheels"
    bl_label = "Refresh Wheels"
    bl_description = "Scan Left/Right collections and report wheel counts"
    def execute(self, ctx):
        info = refresh_wheels(ctx)
        msg = f"L {info['L']} | R {info['R']} | Total {info['total']} / Expected {info['want']}"
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
        self.report({'INFO'}, f"Wheel radius = {r:.4f} m")
        return {'FINISHED'}

# The rest are frozen per your note; keeping stubs to keep panel functional.
from .feasibility import check
from .rpm import build

class ValidatePath(bpy.types.Operator):
    bl_idname = "robo.validate_path"
    bl_label = "Validate Path"
    bl_description = "Basic check of chassis animation"
    def execute(self, ctx):
        res = check(ctx)
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
