# io_export.py — CSV & Keyframe export operators

import bpy
import json
from math import pi
from .utils import _driver_key, _linerp

# ---------------------- CSV Export ----------------------
class SG_OT_ExportCSV(bpy.types.Operator):
    bl_idname = "segway.export_csv"
    bl_label = "Export CSV"
    bl_description = "Export t, x, y, yaw, thetaR/L, rateR/L using chosen sampling and units"

    def execute(self, context):
        P = context.scene.sg_props
        d = bpy.app.driver_namespace.get(_driver_key())
        if not d:
            self.report({'ERROR'}, "Build Cache first (and pass validation).")
            return {'CANCELLED'}

        # Robust fallbacks if properties aren’t added yet
        path = bpy.path.abspath(getattr(P, "csv_path", "//robot_anim.csv"))
        fps = d['fps']; n = len(d['thetaL'])

        angle_unit = getattr(P, "angle_unit", "RAD")
        angrate_unit = getattr(P, "angrate_unit", "RPM")
        length_unit = getattr(P, "length_unit", "M")
        sample_mode = getattr(P, "sample_mode", "FRAME")
        fixed_rate  = getattr(P, "fixed_rate", 100)

        ang_k = (1.0 if angle_unit == 'RAD' else 180.0/pi)
        if   angrate_unit == 'RPM': rate_k = 60.0/(2.0*pi)
        elif angrate_unit == 'RPS': rate_k = 1.0/(2.0*pi)
        else:                       rate_k = 180.0/pi

        len_k = (1.0 if length_unit == 'M' else 100.0)

        rows = []
        if sample_mode == 'FRAME':
            for i in range(n):
                t = i / fps
                thL = d['thetaL'][i]; thR = d['thetaR'][i]
                x = d['x'][i]*len_k; y = d['y'][i]*len_k
                yaw = d['yaw'][i] * (180.0/pi if angle_unit == 'DEG' else 1.0)
                if i == 0:
                    rateR = rateL = 0.0
                else:
                    dthR = d['thetaR'][i] - d['thetaR'][i-1]
                    dthL = d['thetaL'][i] - d['thetaL'][i-1]
                    rateR = (dthR*fps)*rate_k; rateL = (dthL*fps)*rate_k
                rows.append((t,x,y,yaw, thR*ang_k, thL*ang_k, rateR, rateL))
        else:
            try:
                hz = float(fixed_rate)
            except Exception:
                hz = 100.0
            hz = max(1.0, hz); dt = 1.0/hz; total = (n-1)/fps; k = 0
            while True:
                t = k*dt
                if t > total + 1e-9: break
                idx = t*fps
                thR = _linerp(d['thetaR'], idx); thL = _linerp(d['thetaL'], idx)
                x = _linerp(d['x'], idx)*len_k; y = _linerp(d['y'], idx)*len_k
                yaw = _linerp(d['yaw'], idx) * (180.0/pi if angle_unit == 'DEG' else 1.0)
                if k == 0:
                    rateR = rateL = 0.0
                else:
                    thRprev = rows[-1][4]/ang_k; thLprev = rows[-1][5]/ang_k
                    rateR = ((thR-thRprev)/dt)*rate_k; rateL = ((thL-thLprev)/dt)*rate_k
                rows.append((t,x,y,yaw, thR*ang_k, thL*ang_k, rateR, rateL))
                k += 1

        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write("# Track width is center-to-center of left/right wheels.\n")
                f.write(f"# track_width_m={getattr(P,'track_width',0.0):.6f}, tire_spacing_m={getattr(P,'tire_spacing',0.0):.6f}, "
                        f"swap_lr={getattr(P,'swap_lr',False)}, wheel_forward_invert={getattr(P,'wheel_forward_invert',False)}\n")
                f.write(f"t,{('x_m' if length_unit=='M' else 'x_cm')},{('y_m' if length_unit=='M' else 'y_cm')},"
                        f"yaw_{angle_unit.lower()},thetaR_{angle_unit.lower()},thetaL_{angle_unit.lower()},"
                        f"rateR_{angrate_unit.lower()},rateL_{angrate_unit.lower()}\n")
                for r in rows:
                    f.write("{:.6f},{:.6f},{:.6f},{:.9f},{:.9f},{:.9f},{:.4f},{:.4f}\n".format(*r))
        except Exception as e:
            self.report({'ERROR'}, f"Failed to write CSV: {e}")
            return {'CANCELLED'}

        self.report({'INFO'}, f"Wrote {path} ({len(rows)} samples)")
        return {'FINISHED'}


# ---------------------- Keyframe Export (CSV/JSON) ----------------------
def _collect_keyframes(obj, fmin, fmax):
    frames = set(); ad = obj.animation_data
    if ad and ad.action:
        for fc in ad.action.fcurves:
            if fc.data_path in ("location","rotation_euler","rotation_quaternion"):
                for kp in fc.keyframe_points:
                    fr = int(round(kp.co[0]))
                    if fmin <= fr <= fmax:
                        frames.add(fr)
    return sorted(frames)

class SG_OT_ExportKeyframes(bpy.types.Operator):
    bl_idname = "segway.export_keyframes"
    bl_label = "Export Keyframes"

    def execute(self, context):
        scn = context.scene
        P = scn.sg_props
        ch = P.chassis
        if not ch:
            self.report({'ERROR'}, "Assign the Chassis.")
            return {'CANCELLED'}

        d = bpy.app.driver_namespace.get(_driver_key())
        if not d:
            self.report({'ERROR'}, "Build Cache first (and pass validation).")
            return {'CANCELLED'}

        f0 = d['f0']; f1 = f0 + len(d['thetaL']) - 1; fps = d['fps']

        # robust props (defaults if missing)
        other_export_path   = bpy.path.abspath(getattr(P, "other_export_path", "//anim_keyframes.csv"))
        other_export_format = getattr(P, "other_export_format", "CSV")
        other_angle_unit    = getattr(P, "other_angle_unit", "RAD")

        kf = _collect_keyframes(ch, scn.frame_start, scn.frame_end) or [scn.frame_start, scn.frame_end]
        kf = [f for f in kf if f0 <= f <= f1]
        if not kf:
            self.report({'ERROR'}, "No keyed frames fall inside the cache/frame range.")
            return {'CANCELLED'}

        ang_k = 1.0 if other_angle_unit == 'RAD' else (180.0/pi)

        rows = []
        deps = context.evaluated_depsgraph_get()
        prev = scn.frame_current
        try:
            for f in kf:
                scn.frame_set(f); deps.update()
                mw = ch.matrix_world
                loc = mw.translation; eul = mw.to_euler('XYZ'); quat = mw.to_quaternion()
                idx = f - f0
                thR = d['thetaR'][idx]*ang_k; thL = d['thetaL'][idx]*ang_k
                rows.append({
                    "frame": int(f), "fps": float(fps), "time_s": float((f - f0)/fps),
                    "x": float(loc.x), "y": float(loc.y), "z": float(loc.z),
                    "euler_x": float(eul.x*ang_k), "euler_y": float(eul.y*ang_k), "euler_z": float(eul.z*ang_k),
                    "quat_w": float(quat.w), "quat_x": float(quat.x), "quat_y": float(quat.y), "quat_z": float(quat.z),
                    "thetaR": float(thR), "thetaL": float(thL)
                })
        finally:
            scn.frame_set(prev); deps.update()

        try:
            if other_export_format == 'CSV':
                with open(other_export_path, "w", encoding="utf-8") as f:
                    f.write(f"# keyframes_only=1, angle_unit={other_angle_unit}\n")
                    f.write("frame,fps,time_s,x,y,z,euler_x,euler_y,euler_z,quat_w,quat_x,quat_y,quat_z,thetaR,thetaL\n")
                    for r in rows:
                        f.write("{frame},{fps:.6f},{time_s:.6f},{x:.6f},{y:.6f},{z:.6f},{euler_x:.9f},{euler_y:.9f},{euler_z:.9f},"
                                "{quat_w:.9f},{quat_x:.9f},{quat_y:.9f},{quat_z:.9f},{thetaR:.9f},{thetaL:.9f}\n".format(**r))
            else:
                out = {
                    "meta": {
                        "angle_unit": other_angle_unit, "fps": fps,
                        "swap_lr": getattr(P, "swap_lr", False),
                        "wheel_forward_invert": getattr(P, "wheel_forward_invert", False),
                        "frame_start": int(scn.frame_start), "frame_end": int(scn.frame_end)
                    },
                    "samples": rows
                }
                with open(other_export_path, "w", encoding="utf-8") as f:
                    json.dump(out, f, indent=2)
        except Exception as e:
            self.report({'ERROR'}, f"Failed to write: {e}")
            return {'CANCELLED'}

        self.report({'INFO'}, f"Keyframes exported to {other_export_path} ({len(rows)} rows)")
        return {'FINISHED'}


# ---------------------- Registration ----------------------
classes = (
    SG_OT_ExportCSV,
    SG_OT_ExportKeyframes,
)

def register():
    from bpy.utils import register_class
    for c in classes:
        register_class(c)

def unregister():
    from bpy.utils import unregister_class
    for c in reversed(classes):
        unregister_class(c)
