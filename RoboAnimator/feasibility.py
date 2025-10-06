import bpy
from math import pi, sin, cos, atan2

# ---------- props (V8-like semantics) ----------
class FeasProps(bpy.types.PropertyGroup):
    body_fwd: bpy.props.EnumProperty(
        name="Body Forward Axis",
        # If '+' ids error on your Blender, switch to safe ids (PX/NX/PY/NY) in both code and UI.
        items=[('+X','Local +X',''), ('-X','Local -X',''),
               ('+Y','Local +Y',''), ('-Y','Local -Y','')],
        default='+Y',
    )
    side_tol_ms: bpy.props.FloatProperty(
        name="Sideways Tolerance (m/s)", default=0.02, min=0.0, precision=6
    )
    path_type: bpy.props.EnumProperty(
        name="Autocorrect Mode",
        items=[('SEASE','S-Curve',''), ('LINEAR','Linear','')],
        default='SEASE',
    )
    tan_scale: bpy.props.FloatProperty(
        name="Tangent scale (S-Ease)", default=1.0, min=0.0, precision=3
    )
    rot_frac: bpy.props.FloatProperty(
        name="Rotation fraction (Linear)", default=0.25, min=0.0, max=1.0, precision=3
    )

def register_feas_props():
    bpy.types.Scene.ra_feas = bpy.props.PointerProperty(type=FeasProps)

def unregister_feas_props():
    if hasattr(bpy.types.Scene, "ra_feas"):
        del bpy.types.Scene.ra_feas

# ---------- math helpers ----------
def _wrap(a):
    while a <= -pi: a += 2*pi
    while a >   pi: a -= 2*pi
    return a

def _unwrap(prev, cur):
    d = cur - prev
    while d >  pi: cur -= 2*pi; d = cur - prev
    while d < -pi: cur += 2*pi; d = cur - prev
    return cur

def yaw_to_heading(F, yaw_world):
    fa = F.body_fwd
    if   fa=='+X': return _wrap(yaw_world)
    elif fa=='-X': return _wrap(yaw_world + pi)
    elif fa=='+Y': return _wrap(yaw_world + pi/2.0)
    else:          return _wrap(yaw_world - pi/2.0)  # -Y

def heading_to_yaw(F, heading):
    fa = F.body_fwd
    if   fa=='+X': return _wrap(heading)
    elif fa=='-X': return _wrap(heading - pi)
    elif fa=='+Y': return _wrap(heading - pi/2.0)
    else:          return _wrap(heading + pi/2.0)

def _body_basis_from_yaw(theta, forward_axis):
    c, s = cos(theta), sin(theta)
    if   forward_axis=='+Y': fwd=(-s, c)
    elif forward_axis=='-Y': fwd=( s,-c)
    elif forward_axis=='-X': fwd=(-c,-s)
    else:                    fwd=( c, s)  # +X
    lat = (-fwd[1], fwd[0])
    return fwd, lat

# ---------- validation (V8 mid-step lateral speed) ----------
def analyze(ctx):
    P = ctx.scene.ra_props; ch = P.chassis
    if not ch: return {"ok": False, "msg": "Assign the chassis."}
    scn = ctx.scene
    if scn.frame_end <= scn.frame_start:
        return {"ok": False, "msg": "Scene frame range invalid."}

    F   = ctx.scene.ra_feas
    fps = scn.render.fps / scn.render.fps_base
    dt  = 1.0 / float(fps)
    f0, f1 = scn.frame_start, scn.frame_end

    scn.frame_set(f0)
    prev_loc = ch.matrix_world.translation.copy()
    prev_yaw = ch.matrix_world.to_euler('XYZ').z

    violations = 0
    v_frames   = []
    for f in range(f0 + 1, f1 + 1):
        scn.frame_set(f)
        loc = ch.matrix_world.translation
        yaw = _unwrap(prev_yaw, ch.matrix_world.to_euler('XYZ').z)
        yaw_mid = _wrap((prev_yaw + yaw) * 0.5)

        dp_x = loc.x - prev_loc.x
        dp_y = loc.y - prev_loc.y

        _fwd, lat = _body_basis_from_yaw(yaw_mid, F.body_fwd)
        # lateral meters per frame
        dy = dp_x * lat[0] + dp_y * lat[1]
        # compare lateral speed (m/s)
        if abs(dy) > (F.side_tol_ms * dt):
            violations += 1
            v_frames.append(f)

        prev_loc = loc.copy()
        prev_yaw = yaw

    ok  = (violations == 0)
    msg = "Tol {:.6f} m/s | Violations {}".format(F.side_tol_ms, violations)
    return {"ok": ok, "msg": msg, "violations": violations, "violation_frames": v_frames, "side_tol": F.side_tol_ms}

# ---------- fcurve utils ----------
def _get_action_and_fcurves(obj):
    ad = obj.animation_data
    if not ad or not ad.action:
        return None, []
    act = ad.action
    fcs = [fc for fc in act.fcurves if fc.data_path in ("location","rotation_euler")]
    return act, fcs

def _snapshot_fcurves(fcurves):
    snap = []
    for fc in fcurves:
        keys = []
        for kp in fc.keyframe_points:
            keys.append((kp.co.x, kp.co.y, kp.interpolation,
                         kp.handle_left_type, kp.handle_right_type,
                         (kp.handle_left.x, kp.handle_left.y),
                         (kp.handle_right.x, kp.handle_right.y)))
        snap.append((fc.data_path, fc.array_index, keys))
    return snap

def _restore_fcurves(obj, snap):
    ad = obj.animation_data
    if not ad or not ad.action:
        return False
    act = ad.action
    for fc in list(act.fcurves):
        if fc.data_path in ("location","rotation_euler"):
            act.fcurves.remove(fc)
    for data_path, idx, keys in snap:
        fc = act.fcurves.new(data_path=data_path, index=idx)
        for co_x, co_y, interp, hl, hr, hL, hR in keys:
            kp = fc.keyframe_points.insert(frame=co_x, value=co_y)
            kp.interpolation = interp
            kp.handle_left_type  = hl
            kp.handle_right_type = hr
            kp.handle_left.x,  kp.handle_left.y  = hL
            kp.handle_right.x, kp.handle_right.y = hR
    return True

def _ensure_action(obj):
    if not obj.animation_data:
        obj.animation_data_create()
    if not obj.animation_data.action:
        obj.animation_data.action = bpy.data.actions.new(obj.name + "_Action")
    return obj.animation_data.action

def _segment_frames_from_location(act):
    frames = set()
    for fc in act.fcurves:
        if fc.data_path == "location":
            for kp in fc.keyframe_points:
                frames.add(int(round(kp.co[0])))
    return sorted(frames)

def _loc_at_frame(obj, f):
    bpy.context.scene.frame_set(f)
    mw = obj.matrix_world
    t = mw.translation
    return float(t.x), float(t.y)

def _yaw_at_frame(obj, f):
    bpy.context.scene.frame_set(f)
    return float(obj.matrix_world.to_euler('XYZ').z)

def _insert(act, path, idx, fr, val, interp):
    fc = None
    for fcurve in act.fcurves:
        if fcurve.data_path == path and fcurve.array_index == idx:
            fc = fcurve; break
    if fc is None:
        fc = act.fcurves.new(path, index=idx)
    kp = fc.keyframe_points.insert(frame=fr, value=val)
    kp.interpolation = interp

# ---------- constraint/NLA mute helpers ----------
def _store_and_set_constraints(obj, mute: bool):
    saved = []
    for c in getattr(obj, "constraints", []):
        if c.type in {"FOLLOW_PATH","COPY_LOCATION","COPY_ROTATION","CHILD_OF","LIMIT_LOCATION","LIMIT_ROTATION","TRANSFORM"}:
            saved.append((c, getattr(c, "mute", False)))
            c.mute = mute
    return saved

def _store_and_set_nla_tracks(obj, mute: bool):
    saved = []
    ad = obj.animation_data
    if not ad:
        return saved
    for tr in getattr(ad, "nla_tracks", []):
        saved.append((tr, tr.mute))
        tr.mute = mute
    return saved

# ---------- autocorrect (parameters + visible interp) ----------
def autocorrect(ctx):
    scn = ctx.scene; ch = scn.ra_props.chassis
    if not ch: return {"ok": False, "msg": "Assign the chassis."}
    act, fcs = _get_action_and_fcurves(ch)
    if act is None: return {"ok": False, "msg": "Chassis has no action."}

    ns = bpy.app.driver_namespace
    if "TRA_AUTOBK" not in ns:
        ns["TRA_AUTOBK"] = _snapshot_fcurves(fcs)

    if scn.ra_feas.path_type == 'SEASE':
        for fc in fcs:
            if fc.data_path == "location":
                for kp in fc.keyframe_points:
                    kp.interpolation = 'BEZIER'
                    kp.handle_left_type = 'AUTO_CLAMPED'
                    kp.handle_right_type= 'AUTO_CLAMPED'
    else:
        for fc in fcs:
            if fc.data_path in ("location","rotation_euler"):
                for kp in fc.keyframe_points:
                    kp.interpolation = 'LINEAR'

    ns["TRA_AUTO"] = {"mode": scn.ra_feas.path_type,
                      "tan_scale": scn.ra_feas.tan_scale,
                      "rot_frac": scn.ra_feas.rot_frac}
    return {"ok": True, "msg": "Autocorrect applied (parameters set)."}

def revert_autocorrect(ctx):
    scn = ctx.scene; ch = scn.ra_props.chassis
    if not ch: return {"ok": False, "msg": "Assign the chassis."}
    ns = bpy.app.driver_namespace
    snap = ns.get("TRA_AUTOBK")
    mutes = ns.get("TRA_BAKE_MUTES")

    msg = "Nothing to revert."
    ok = True

    if snap:
        act, _ = _get_action_and_fcurves(ch)
        if act is None:
            return {"ok": False, "msg": "No action to restore."}
        ok = _restore_fcurves(ch, snap)
        msg = "Autocorrect reverted." if ok else "Revert failed."
        ns.pop("TRA_AUTOBK", None)
        ns.pop("TRA_AUTO", None)

    if mutes:
        for c, prev in mutes.get("constraints", []):
            try: c.mute = prev
            except: pass
        for tr, prev in mutes.get("nla", []):
            try: tr.mute = prev
            except: pass
        ns.pop("TRA_BAKE_MUTES", None)

    return {"ok": ok, "msg": msg}

# ---------- bake (V8 behavior, wheels ignored) ----------
def _s_ease(u, k):
    u = max(0.0, min(1.0, u))
    u = 0.5 + (u - 0.5) * (k if k > 0.0 else 1.0)
    s = 3*u*u - 2*u*u*u
    return max(0.0, min(1.0, s))

def bake_motion(ctx):
    scn = ctx.scene; ch = scn.ra_props.chassis
    if not ch: return {"ok": False, "msg": "Assign the chassis."}
    F = scn.ra_feas
    act = _ensure_action(ch)

    # backup once
    act0, fcs0 = _get_action_and_fcurves(ch)
    ns = bpy.app.driver_namespace
    if "TRA_AUTOBK" not in ns:
        ns["TRA_AUTOBK"] = _snapshot_fcurves(fcs0)

    seg = _segment_frames_from_location(act)
    if len(seg) < 2:
        seg = [scn.frame_start, scn.frame_end]
        if seg[0] >= seg[1]:
            return {"ok": False, "msg": "No segment to bake."}

    # clear channels
    for fc in list(act.fcurves):
        if fc.data_path in ("location","rotation_euler"):
            act.fcurves.remove(fc)

    # write keys from current evaluated motion, then mute constraint/NLA so keys win
    for i in range(len(seg) - 1):
        f0, f1 = seg[i], seg[i+1]
        if f1 <= f0: continue
        n = f1 - f0

        x0, y0 = _loc_at_frame(ch, f0)
        x1, y1 = _loc_at_frame(ch, f1)
        dx, dy = (x1 - x0), (y1 - y0)
        heading = atan2(dy, dx)
        yaw = heading_to_yaw(F, heading)

        if F.path_type == 'LINEAR':
            n_rot = int(round(F.rot_frac * n))
            n_rot = max(0, min(n, n_rot))
            yaw0 = _yaw_at_frame(ch, f0)

            # rotate in place
            for t in range(n_rot + 1):
                fr = f0 + t
                a = t / max(1, n_rot) if n_rot > 0 else 1.0
                y = _unwrap(yaw0, yaw0 + (yaw - yaw0) * a)
                _insert(act, "location", 0, fr, x0, "LINEAR")
                _insert(act, "location", 1, fr, y0, "LINEAR")
                _insert(act, "rotation_euler", 2, fr, y,  "LINEAR")

            # translate with fixed yaw
            n_tr = n - n_rot
            for t in range(1, n_tr + 1):
                fr = f0 + n_rot + t
                s = t / max(1, n_tr)
                _insert(act, "location", 0, fr, x0 + dx * s, "LINEAR")
                _insert(act, "location", 1, fr, y0 + dy * s, "LINEAR")
                _insert(act, "rotation_euler", 2, fr, yaw,   "LINEAR")

        else:  # SEASE
            k = F.tan_scale
            for t in range(n + 1):
                fr = f0 + t
                s = _s_ease(t / float(n), k)
                _insert(act, "location", 0, fr, x0 + dx * s, "BEZIER")
                _insert(act, "location", 1, fr, y0 + dy * s, "BEZIER")
                _insert(act, "rotation_euler", 2, fr, yaw,   "BEZIER")

    # tidy handles
    if F.path_type == 'SEASE':
        for fc in act.fcurves:
            for kp in fc.keyframe_points:
                kp.interpolation = 'BEZIER'
                kp.handle_left_type  = 'AUTO_CLAMPED'
                kp.handle_right_type = 'AUTO_CLAMPED'
    else:
        for fc in act.fcurves:
            for kp in fc.keyframe_points:
                kp.interpolation = 'LINEAR'

    # mute sources that would override keys
    saved_c = _store_and_set_constraints(ch, True)
    saved_n = _store_and_set_nla_tracks(ch, True)
    ns["TRA_BAKE_MUTES"] = {"constraints": saved_c, "nla": saved_n}

    return {"ok": True, "msg": "Bake complete. Constraints/NLA muted so keys take effect."}

# Back-compat for Validate Path
def check(ctx):
    return analyze(ctx)
