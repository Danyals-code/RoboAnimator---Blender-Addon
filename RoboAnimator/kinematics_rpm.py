# kinematics_rpm.py — motion feasibility, cache, drivers, bake/clear
# Revised to support explicit 2W/4W wheel pointers with legacy-collection fallback
# (No other files required to change.)

import bpy
from bpy.types import Operator
from math import pi, sin, cos, sqrt, atan2

from .utils import (
    _driver_key, _AXIS_INDEX, _axis_unit, _auto_radius,
    _ensure_xyz_euler, _ensure_quaternion, _obj_rest_quat,
    _col_for_side, _iter_side, _body_basis_from_yaw,
    _wrap, _unwrap,
)

# Accept both legacy ('X','Y','Z') and signed ('+X','-X', etc.) forms
_AXIS_INDEX = {
    'X': 0, 'Y': 1, 'Z': 2,
    '+X': 0, '-X': 0,
    '+Y': 1, '-Y': 1,
    '+Z': 2, '-Z': 2,
    0: 0, 1: 1, 2: 2,   # tolerate numeric forms
}
_AXIS_SIGN = {
    'X': 1, 'Y': 1, 'Z': 1,
    '+X': 1, '-X': -1,
    '+Y': 1, '-Y': -1,
    '+Z': 1, '-Z': -1,
    0: 1, 1: 1, 2: 1,
}


# ───────────────────────────────
# Local wheel resolution helpers
# ───────────────────────────────

def _resolve_wheels(P, side):
    """
    Return a list of wheel objects on the requested side ('L' or 'R').

    Priority:
      1) If P.wheel_system == '4W', use explicit pointers:
         L -> wheel_fl, wheel_rl   |  R -> wheel_fr, wheel_rr
      2) Else use explicit 2W pointers:
         L -> wheel_left           |  R -> wheel_right
      3) Else fall back to legacy collections via _iter_side(P, side)

    This function is self-contained so we don't have to touch other files.
    """
    side = side.upper()
    out = []

    # 1) 4-wheel explicit
    sys = getattr(P, "wheel_system", None)
    if sys == '4W':
        if side == 'L':
            for name in ("wheel_fl", "wheel_rl"):
                o = getattr(P, name, None)
                if isinstance(o, bpy.types.Object):
                    out.append(o)
        else:
            for name in ("wheel_fr", "wheel_rr"):
                o = getattr(P, name, None)
                if isinstance(o, bpy.types.Object):
                    out.append(o)

    # 2) 2-wheel explicit
    if not out:
        name = "wheel_left" if side == 'L' else "wheel_right"
        o = getattr(P, name, None)
        if isinstance(o, bpy.types.Object):
            out.append(o)

    # 3) Legacy collections (original behavior)
    if not out:
        for o in _iter_side(P, side):
            out.append(o)

    # Deduplicate by object name while preserving order
    seen = set()
    dedup = []
    for o in out:
        if o and o.name not in seen:
            seen.add(o.name)
            dedup.append(o)
    return dedup


def _first_wheel(P, side):
    """Return a single representative wheel for a side, preferring explicit pointers."""
    wheels = _resolve_wheels(P, side)
    return wheels[0] if wheels else None


# ---------------------- validation (no sideways slip per frame) ----------------------
def analyze_motion(context):
    P = context.scene.sg_props
    ch = P.chassis
    if not ch:
        raise RuntimeError("Assign the Chassis.")
    if P.track_width <= 0:
        raise RuntimeError("Track width must be > 0.")
    scn = context.scene
    deps = context.evaluated_depsgraph_get()
    if scn.frame_end <= scn.frame_start:
        raise RuntimeError("Scene frame range invalid.")

    fps = scn.render.fps / scn.render.fps_base
    dt = 1.0 / float(fps)
    f0, f1 = scn.frame_start, scn.frame_end
    scn.frame_set(f0); deps.update()
    prev_loc = ch.matrix_world.translation.copy()
    prev_yaw = ch.matrix_world.to_euler('XYZ').z

    violations = 0
    v_frames = []
    for f in range(f0 + 1, f1 + 1):
        scn.frame_set(f); deps.update()
        loc = ch.matrix_world.translation
        yaw = _unwrap(prev_yaw, ch.matrix_world.to_euler('XYZ').z)
        yaw_mid = _wrap((prev_yaw + yaw) * 0.5)

        dp_x = loc.x - prev_loc.x
        dp_y = loc.y - prev_loc.y
        fwd, lat = _body_basis_from_yaw(yaw_mid, P.body_forward_axis)
        dx = dp_x * fwd[0] + dp_y * fwd[1]
        dy = dp_x * lat[0] + dp_y * lat[1]

        if abs(dy) > (P.side_tol * dt):
            violations += 1
            v_frames.append(f)

        prev_loc = loc.copy()
        prev_yaw = yaw

    return {
        'f0': f0, 'fps': fps, 'track': P.track_width,
        'violations': violations, 'violation_frames': v_frames,
        'side_tol': P.side_tol, 'dt': dt
    }

# ---------------------- cache build for wheel drivers ----------------------
def build_cache(context):
    """
    Validate, then integrate kinematics to produce per-frame wheel angles.
    Also enforces wheel RPM and RPM/s limits if set.
    """
    P = context.scene.sg_props
    ch = P.chassis
    if not ch:
        raise RuntimeError("Assign the Chassis.")

    a = analyze_motion(context)
    if a['violations'] > 0:
        raise RuntimeError(
            f"Infeasible motion: {a['violations']} frame(s) exceed sideways tolerance > {a['side_tol']} "
            f"(e.g., frames {a['violation_frames'][:10]})."
        )

    # Prefer explicit wheels for reference; fall back to original collection-first approach
    wL0 = _first_wheel(P, 'L')
    wR0 = _first_wheel(P, 'R')

    # If still nothing, keep legacy behavior to avoid breaking old scenes
    if not wL0 and not wR0:
        colL = _col_for_side(P, 'L')
        colR = _col_for_side(P, 'R')
        wL0 = colL.objects[0] if (colL and len(colL.objects) > 0) else None
        wR0 = colR.objects[0] if (colR and len(colR.objects) > 0) else None

    restL = _obj_rest_quat(wL0) if wL0 else (1.0, 0.0, 0.0, 0.0)
    restR = _obj_rest_quat(wR0) if wR0 else (1.0, 0.0, 0.0, 0.0)

    signL = +1.0 if P.sign_l == 'PLUS' else -1.0
    signR = +1.0 if P.sign_r == 'PLUS' else -1.0
    try:
        if P.wheel_forward_invert:
            signL *= -1.0
            signR *= -1.0
    except Exception:
        pass

    b = P.track_width
    if P.auto_radius:
        # For auto radius, prefer any resolved wheel; fall back to legacy error
        w_ref = wL0 or wR0
        if not w_ref:
            raise RuntimeError("Assign wheel objects (explicit or in Left/Right collections) for auto radius.")
        radius = _auto_radius(w_ref, P.wheel_axis)
    else:
        radius = P.wheel_radius
    if radius <= 0:
        raise RuntimeError("Wheel radius must be > 0.")

    scn = context.scene
    deps = context.evaluated_depsgraph_get()
    fps = scn.render.fps / scn.render.fps_base
    dt = 1.0 / float(fps)
    f0 = scn.frame_start
    f1 = scn.frame_end

    thetaL = [0.0]
    thetaR = [0.0]
    pos_x = []
    pos_y = []
    yaw_z = []

    scn.frame_set(f0); deps.update()
    prev_loc = ch.matrix_world.translation.copy()
    prev_yaw = ch.matrix_world.to_euler('XYZ').z
    pos_x.append(prev_loc.x)
    pos_y.append(prev_loc.y)
    yaw_z.append(prev_yaw)
    tL = tR = 0.0
    rpmL_list = [0.0]
    rpmR_list = [0.0]

    for f in range(f0 + 1, f1 + 1):
        scn.frame_set(f); deps.update()
        loc = ch.matrix_world.translation
        yaw = _unwrap(prev_yaw, ch.matrix_world.to_euler('XYZ').z)
        yaw_mid = _wrap((prev_yaw + yaw) * 0.5)

        dp_x = loc.x - prev_loc.x
        dp_y = loc.y - prev_loc.y
        fwd, _lat = _body_basis_from_yaw(yaw_mid, P.body_forward_axis)
        dx = dp_x * fwd[0] + dp_y * fwd[1]

        if getattr(P, "wheel_forward_invert", False):
            dx = -dx

        dpsi = yaw - prev_yaw
        dsL = dx - 0.5 * b * dpsi
        dsR = dx + 0.5 * b * dpsi
        dthL = (dsL / radius) * signL
        dthR = (dsR / radius) * signR
        tL += dthL
        tR += dthR
        thetaL.append(tL)
        thetaR.append(tR)
        pos_x.append(loc.x)
        pos_y.append(loc.y)
        yaw_z.append(yaw)

        rpmL = (dthL * fps) * 60.0 / (2.0 * pi)
        rpmR = (dthR * fps) * 60.0 / (2.0 * pi)
        rpmL_list.append(rpmL)
        rpmR_list.append(rpmR)

        prev_loc = loc.copy()
        prev_yaw = yaw

    # limits
    max_rpm_limit = float(getattr(P, "max_rpm", 0.0))
    max_arpm_s_limit = float(getattr(P, "max_ang_accel_rpm_s", 0.0))

    if max_rpm_limit > 0.0:
        over = []
        for i, (rL, rR) in enumerate(zip(rpmL_list, rpmR_list)):
            if abs(rL) > max_rpm_limit or abs(rR) > max_rpm_limit:
                over.append(f0 + i)
        if over:
            head = ", ".join(map(str, over[:12])) + (" …" if len(over) > 12 else "")
            raise RuntimeError(f"Wheel RPM limit exceeded (>{max_rpm_limit:.1f} rpm) at frames: {head}")

    if max_arpm_s_limit > 0.0:
        over = []
        for i in range(1, len(rpmL_list)):
            aL = (rpmL_list[i] - rpmL_list[i - 1]) * fps  # rpm per second
            aR = (rpmR_list[i] - rpmR_list[i - 1]) * fps
            if abs(aL) > max_arpm_s_limit or abs(aR) > max_arpm_s_limit:
                over.append(f0 + i)
        if over:
            head = ", ".join(map(str, over[:12])) + (" …" if len(over) > 12 else "")
            raise RuntimeError(f"Wheel angular-accel limit exceeded (>{max_arpm_s_limit:.1f} rpm/s) at frames: {head}")

    data = {
        'f0': f0, 'fps': fps,
        'thetaL': thetaL, 'thetaR': thetaR,
        'x': pos_x, 'y': pos_y, 'yaw': yaw_z,
        'restL': restL, 'restR': restR,
        'radius': radius, 'track': b,
        'max_rpm_L': max(abs(r) for r in rpmL_list),
        'max_rpm_R': max(abs(r) for r in rpmR_list),
        'violations': 0, 'violation_frames': []
    }
    bpy.app.driver_namespace[_driver_key()] = data
    return data

# ---------------------- driver functions (for expressions) ----------------------
def sg_theta(side, frame):
    d = bpy.app.driver_namespace.get(_driver_key())
    if not d:
        return 0.0
    i = int(frame - d['f0'])
    i = max(0, min(i, len(d['thetaL']) - 1))
    return d['thetaL'][i] if side == 'L' else d['thetaR'][i]

def sg_quat_comp(side, frame, comp_index, axis_char):
    d = bpy.app.driver_namespace.get(_driver_key())
    if not d:
        return (1.0, 0.0, 0.0, 0.0)[int(comp_index) % 4]
    th = sg_theta(side, frame)
    ux, uy, uz = _axis_unit(axis_char)
    h = 0.5 * th
    s = sin(h); c = cos(h)
    q_spin = (c, ux * s, uy * s, uz * s)
    q_rest = d['restL'] if side == 'L' else d['restR']
    aw, ax, ay, az = q_rest
    bw, bx, by, bz = q_spin
    q = (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw
    )
    return q[int(comp_index) % 4]

def sg_quat_comp_obj(side, frame, comp_index, axis_char, rw, rx, ry, rz):
    d = bpy.app.driver_namespace.get(_driver_key())
    if not d:
        return (1.0, 0.0, 0.0, 0.0)[int(comp_index) % 4]
    th = sg_theta(side, frame)
    ux, uy, uz = _axis_unit(axis_char)
    h = 0.5 * th
    s = sin(h); c = cos(h)
    q_spin = (c, ux * s, uy * s, uz * s)
    aw, ax, ay, az = (rw, rx, ry, rz)
    bw, bx, by, bz = q_spin
    q = (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw
    )
    return q[int(comp_index) % 4]

# expose helpers in driver namespace (for scripted expressions)
bpy.app.driver_namespace['sg_theta'] = sg_theta
bpy.app.driver_namespace['sg_quat_comp'] = sg_quat_comp
bpy.app.driver_namespace['sg_quat_comp_obj'] = sg_quat_comp_obj

# ---------------------- Operators ----------------------
class SG_OT_ValidateMotion(Operator):
    bl_idname = "segway.validate_motion"
    bl_label = "Validate Motion"
    bl_description = "Check current chassis animation for nonholonomic feasibility (mid-step heading)"

    def execute(self, context):
        try:
            a = analyze_motion(context)
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        if a['violations'] > 0:
            frames = a['violation_frames']
            head = ", ".join(str(f) for f in frames[:12]) + (" …" if len(frames) > 12 else "")
            self.report({'ERROR'}, f"This won't work: {a['violations']} step(s) exceed sideways tolerance > {a['side_tol']} (frames {head}).")
            return {'CANCELLED'}
        self.report({'INFO'}, "Motion is feasible (no slip violations).")
        return {'FINISHED'}

class SG_OT_AttachDrivers(Operator):
    bl_idname = "segway.attach_drivers"
    bl_label = "Attach Drivers"

    def execute(self, context):
        P = context.scene.sg_props
        if not bpy.app.driver_namespace.get(_driver_key()):
            self.report({'ERROR'}, "Build Cache first (and pass validation).")
            return {'CANCELLED'}
        axis_i = _AXIS_INDEX[P.wheel_axis]
        for side in ('L', 'R'):
            for obj in _resolve_wheels(P, side):  # ← use new resolver
                if P.rotation_mode == 'EULER':
                    _ensure_xyz_euler(obj)
                    try:
                        for i in range(4):
                            obj.driver_remove("rotation_quaternion", i)
                    except Exception:
                        pass
                    try:
                        obj.driver_remove("rotation_euler", axis_i)
                    except Exception:
                        pass
                    dcurve = obj.driver_add("rotation_euler", axis_i)
                    dcurve.driver.type = 'SCRIPTED'
                    dcurve.driver.expression = f"sg_theta('{side}', frame)"
                else:
                    _ensure_quaternion(obj)
                    try:
                        obj.driver_remove("rotation_euler", axis_i)
                    except Exception:
                        pass
                    try:
                        for i in range(4):
                            obj.driver_remove("rotation_quaternion", i)
                    except Exception:
                        pass
                    for comp in range(4):
                        dcurve = obj.driver_add("rotation_quaternion", comp)
                        dcurve.driver.type = 'SCRIPTED'
                        rq = _obj_rest_quat(obj)
                        obj['rest_w'], obj['rest_x'], obj['rest_y'], obj['rest_z'] = rq
                        dcurve.driver.expression = f"sg_quat_comp_obj('{side}', frame, {comp}, '{P.wheel_axis}', rw, rx, ry, rz)"
                        var = dcurve.driver.variables.new()
                        var.name = 'rw'; var.targets[0].id = obj; var.targets[0].data_path = '["rest_w"]'
                        var = dcurve.driver.variables.new()
                        var.name = 'rx'; var.targets[0].id = obj; var.targets[0].data_path = '["rest_x"]'
                        var = dcurve.driver.variables.new()
                        var.name = 'ry'; var.targets[0].id = obj; var.targets[0].data_path = '["rest_y"]'
                        var = dcurve.driver.variables.new()
                        var.name = 'rz'; var.targets[0].id = obj; var.targets[0].data_path = '["rest_z"]'
        self.report({'INFO'}, "Drivers attached to wheel objects.")
        return {'FINISHED'}

class SG_OT_BuildCache(Operator):
    bl_idname = "segway.build_cache"
    bl_label = "Build Cache"

    def execute(self, context):
        try:
            d = build_cache(context)
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        self.report({'INFO'}, f"OK | r={d['radius']:.4f} m | track={d['track']:.4f} m | maxRPM L/R {d['max_rpm_L']:.1f}/{d['max_rpm_R']:.1f}")
        return {'FINISHED'}

class SG_OT_Bake(Operator):
    bl_idname = "segway.bake_wheels"
    bl_label = "Bake Wheels"

    def execute(self, context):
        P = context.scene.sg_props
        d = bpy.app.driver_namespace.get(_driver_key())
        if not d:
            self.report({'ERROR'}, "Build Cache first (and pass validation).")
            return {'CANCELLED'}
        f0 = d['f0']
        f1 = f0 + len(d['thetaL']) - 1
        axis_i = _AXIS_INDEX[P.wheel_axis]
        ux, uy, uz = _axis_unit(P.wheel_axis)

        for side in ('L', 'R'):
            arr = d['thetaL'] if side == 'L' else d['thetaR']
            rest = d['restL'] if side == 'L' else d['restR']
            for obj in _resolve_wheels(P, side):  # ← use new resolver
                if P.rotation_mode == 'EULER':
                    _ensure_xyz_euler(obj)
                    try:
                        obj.driver_remove("rotation_euler", axis_i)
                    except Exception:
                        pass
                else:
                    _ensure_quaternion(obj)
                    try:
                        for i in range(4):
                            obj.driver_remove("rotation_quaternion", i)
                    except Exception:
                        pass

                for f in range(f0, f1 + 1):
                    i = f - f0
                    th = arr[i]
                    if P.rotation_mode == 'EULER':
                        obj.rotation_euler[axis_i] = th
                        obj.keyframe_insert("rotation_euler", frame=f, index=axis_i)
                    else:
                        half = 0.5 * th
                        s = sin(half); c = cos(half)
                        q_spin = (c, ux * s, uy * s, uz * s)
                        aw, ax, ay, az = rest
                        bw, bx, by, bz = q_spin
                        q = (
                            aw * bw - ax * bx - ay * by - az * bz,
                            aw * bx + ax * bw + ay * bz - az * by,
                            aw * by - ax * bz + ay * bw + az * bx,
                            aw * bz + ax * by - ay * bx + az * bw
                        )
                        rq = obj.rotation_quaternion
                        rq[0], rq[1], rq[2], rq[3] = q
                        for comp in range(4):
                            obj.keyframe_insert("rotation_quaternion", frame=f, index=comp)
        self.report({'INFO'}, f"Baked {f0}..{f1} to keyframes.")
        return {'FINISHED'}

class SG_OT_Clear(Operator):
    bl_idname = "segway.clear"
    bl_label = "Clear Drivers/Keyframes"
    bl_description = "Remove wheel rotation drivers AND their rotation keyframes (Euler & Quaternion) on all wheel objects."

    def execute(self, context):
        P = context.scene.sg_props
        axis_i = _AXIS_INDEX[P.wheel_axis]
        removed_any = False
        for side in ('L', 'R'):
            for obj in _resolve_wheels(P, side):  # ← use new resolver
                try:
                    obj.driver_remove("rotation_euler", axis_i); removed_any = True
                except Exception:
                    pass
                try:
                    for i in range(4):
                        obj.driver_remove("rotation_quaternion", i); removed_any = True
                except Exception:
                    pass
                ad = obj.animation_data
                if ad and ad.action:
                    to_del = [
                        fc for fc in ad.action.fcurves
                        if (fc.data_path == "rotation_euler" and fc.array_index == axis_i)
                        or fc.data_path == "rotation_quaternion"
                    ]
                    for fc in to_del:
                        try:
                            ad.action.fcurves.remove(fc); removed_any = True
                        except Exception:
                            pass
                if P.rotation_mode == 'EULER':
                    try:
                        obj.rotation_euler[axis_i] = 0.0
                    except Exception:
                        pass
                else:
                    try:
                        rq = obj.rotation_quaternion
                        rq[0], rq[1], rq[2], rq[3] = 1.0, 0.0, 0.0, 0.0
                    except Exception:
                        pass
        self.report({'INFO'} if removed_any else {'WARNING'},
                    "Cleared drivers & rotation keyframes" if removed_any else "Nothing to clear on wheel objects.")
        return {'FINISHED'}

# ---------------------- Registration ----------------------
classes = (
    SG_OT_ValidateMotion,
    SG_OT_AttachDrivers,
    SG_OT_BuildCache,
    SG_OT_Bake,
    SG_OT_Clear,
)

def register():
    from bpy.utils import register_class
    for c in classes:
        register_class(c)

def unregister():
    from bpy.utils import unregister_class
    for c in reversed(classes):
        unregister_class(c)
