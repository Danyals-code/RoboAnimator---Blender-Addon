import bpy
from .selection import refresh_wheels

class MainPanel(bpy.types.Panel):
    bl_label = "True RoboAnimator"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"

    @staticmethod
    def _sec(layout, flag, title):
        p = bpy.context.scene.ra_props
        box = layout.box()
        row = box.row()
        open_ = getattr(p, flag)
        row.prop(p, flag, text="", icon='TRIA_DOWN' if open_ else 'TRIA_RIGHT', emboss=False)
        row.label(text=title)
        return box if open_ else None

    def draw(self, ctx):
        layout = self.layout
        p = ctx.scene.ra_props
        f = ctx.scene.ra_feas
        layout.use_property_split = True
        layout.use_property_decorate = False

        b = self._sec(layout, "show_instr", "Instructions")
        if b:
            c = b.column(align=True)
            c.label(text="1) Selection and calibration.")
            c.label(text="2) Feasibility setup.")
            c.label(text="3) Validate motion, build cache.")
            c.label(text="4) Attach or bake.")

        b = self._sec(layout, "show_selcal", "Selection and calibration")
        if b:
            col = b.column(align=True)
            col.prop(p, "total_wheels")
            col.separator()
            col.prop(p, "chassis")
            col.prop(p, "wheels_l")
            col.prop(p, "wheels_r")
            col.separator()
            info = refresh_wheels(ctx)
            status = "Found  L {} | R {} | Total {} / Expected {}".format(
                info['L'], info['R'], info['total'], info['want']
            )
            badges = []
            if info['side_mismatch']:
                badges.append("L!=R")
            if info['odd_found']:
                badges.append("odd total")
            if badges:
                status = status + " [" + ", ".join(badges) + "]"
            row = col.row(align=True)
            row.label(text=status, icon=('ERROR' if info['mismatch'] else 'INFO'))

            col.separator()
            col.prop(p, "wheel_axis", text="Wheel Rotation Axis")
            col.separator()
            col.prop(p, "wheel_r_m", text="Wheel Radius (m)")
            col.prop(p, "track_m",   text="Track Width (m)")
            col.prop(p, "max_rpm",   text="Max Wheel Speed (RPM)")
            col.prop(p, "max_rpm_s", text="Max Wheel Accel (RPM/s)")
            col.separator()
            col.operator("robo.auto_radius", text="Auto-Detect Radius", icon='TRACKING_FORWARDS')

        # Feasibility
        box = layout.box()
        box.label(text="Feasibility", icon='MOD_SMOOTH')
        col = box.column(align=True)
        col.prop(f, "body_fwd", text="Body Forward Axis")
        col.prop(f, "side_tol_ms", text="Sideways tolerance (m/s)")
        col.separator()
        col.prop(f, "path_type", text="Autocorrect Path Type")
        row = col.row(align=True)
        row.prop(f, "tan_scale", text="Tangent scale (S-Ease)")
        row.prop(f, "rot_frac",  text="Rotation fraction (Linear)")
        col.separator()
        row = col.row(align=True)
        row.operator("robo.validate_motion", text="Validate Motion", icon='CHECKMARK')
        row.operator("robo.autocorrect_path", text="Autocorrect", icon='MOD_CURVE')
        row.operator("robo.revert_autocorrect", text="Revert", icon='LOOP_BACK')

        b = self._sec(layout, "show_ops", "Operations")
        if b:
            row = b.row(align=True)
            row.operator("robo.validate_path", icon='INFO')
            row.operator("robo.build_cache", icon='IMPORT')
            row.operator("robo.attach_drivers", icon='CONSTRAINT')
