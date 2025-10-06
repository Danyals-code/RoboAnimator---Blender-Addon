import bpy
from math import pi, sin, cos

# ---------- properties (separate from selection) ----------

_AXIS_ITEMS = [
    ('PX', '+X', ''), ('NX', '-X', ''),
    ('PY', '+Y', ''), ('NY', '-Y', ''),
    ('PZ', '+Z', ''), ('NZ', '-Z', ''),
]

class FeasProps(bpy.types.PropertyGroup):
    body_fwd: bpy.props.EnumProperty(
        name="Body Forward Axis",
        items=_AXIS_ITEMS,
        default='PY',
    )
    side_tol_ms: bpy.props.FloatProperty(  # m/s like V8
        name="Sideways tolerance (m/s)",
        description="Max allowed lateral speed",
        default=0.02, min=0.0, precision=6
    )
    path_type: bpy.props.EnumProperty(
        name="Autocorrect Path Type",
        items=[('S', 'S-Curve', ''), ('LIN', 'Linear', '')],
        default='S'
    )
    tan_scale: bpy.props.FloatProperty(
        name="Tangent scale (S-Ease)",
        description="Scale for S-curve tangents",
        default=1.0, min=0.0, precision=3
    )
    rot_frac: bpy.props.FloatProperty(
        name="Rotation fraction (Linear)",
        description="Fraction of segment used for in-place rotation in linear mode",
        default=0.25, min=0.0, max=1.0, precision=3
    )

def register_feas_props():
    bpy.types.Scene.ra_feas = bpy.props.PointerProperty(type=FeasProps)

def unregister_feas_props():
    if hasattr(bpy.types.Scene, "ra_feas"):
        del bpy.types.Scene.ra_feas

# ---------- helpers ----------

def _yaw_to_heading(axis_id, yaw_world):
    # Map world yaw to robot-forward heading in XY.
    if axis_id == 'PX': return yaw_world
    if axis_id == 'NX': return yaw_world + pi
    if axis_id == 'PY': return yaw_world + pi/2.0
    if axis_id == 'NY': return yaw_world - pi/2.0
    # Z-forward fallback uses +Y mapping
    return yaw_world + pi/2.0

def _sample_pose(ctx, frame):
    scn = ctx.scene
    scn.frame_set(frame)
    ch = scn.ra_props.chassis
    mw = ch.matrix_world
    loc = mw.translation
    yaw = mw.to_euler('XYZ').z
    return float(loc.x), float(loc.y), float(yaw)

def _get_action_and_fcurves(obj):
    ad = obj.animation_data
    if not ad or not ad.action:
        return None, []
    act = ad.action
    fcs = [fc for fc in act.fcurves if fc.data_path in ("location", "rotation_euler")]
    return act, fcs

def _snapshot_fcurves(fcurves):
    snap = []
    for fc in fcurves:
        keys = []
        for kp in fc.keyframe_points:
            keys.append((
                kp.co.x, kp.co.y,
                kp.interpolation,
                kp.handle_left_type, kp.handle_right_type,
                (kp.handle_left.x, kp.handle_left.y),
                (kp.handle_right.x, kp.handle_right.y),
            ))
        snap.append((fc.data_path, fc.array_index, keys))
    return snap

def _restore_fcurves(obj, snap):
    ad = obj.animation_data
    if not ad or not ad.action:
        return False
    act = ad.action
    # clear
    for fc in list(act.fcurves):
        if fc.data_path in ("location", "rotation_euler"):
            act.fcurves.remove(fc)
    # rebuild
    for data_path, idx, keys in snap:
        fc = act.fcurves.new(data_path=data_path, index=idx)
        for co_x, co_y, interp, hl_type, hr_type, hl, hr in keys:
            kp = fc.keyframe_points.insert(frame=co_x, value=co_y)
            kp.interpolation = interp
            kp.handle_left_type = hl_type
            kp.handle_right_type = hr_type
            kp.handle_left.x, kp.handle_left.y = hl
            kp.handle_right.x, kp.handle_right.y = hr
    return True

def _apply_s_curve(fcurves, tan_scale: float):
    # Smooth X/Y location with Bezier AUTO_CLAMPED
    for fc in fcurves:
        if fc.data_path != "location":
            continue
        for kp in fc.keyframe_points:
            kp.interpolation = 'BEZIER'
            kp.handle_left_type = 'AUTO_CLAMPED'
            kp.handle_right_type = 'AUTO_CLAMPED'
    # tan_scale kept for bake-time shaping.

def _apply_linear(fcurves):
    # Linear X/Y location
    for fc in fcurves:
        if fc.data_path != "location":
            continue
        for kp in fc.keyframe_points:
            kp.interpolation = 'LINEAR'

# ---------- validation ----------

def analyze(ctx):
    scn = ctx.scene
    P = scn.ra_props
    F = scn.ra_feas
    ch = P.chassis
    if not ch:
        return {"ok": False, "msg": "Assign the chassis."}
    if scn.frame_end <= scn.frame_start:
        return {"ok": False, "msg": "Scene frame range invalid."}

    fps = scn.render.fps / scn.render.fps_base
    dt = 1.0 / float(fps)
    f0, f1 = scn.frame_start, scn.frame_end

    x0, y0, yaw0 = _sample_pose(ctx, f0)
    h0 = _yaw_to_heading(F.body_fwd, yaw0)

    max_side_ms = 0.0
    viol_frames = []

    for f in range(f0 + 1, f1 + 1):
        x1, y1, yaw1 = _sample_pose(ctx, f)
        h1 = _yaw_to_heading(F.body_fwd, yaw1)

        # mid-step heading
        h_mid = 0.5 * (h0 + h1)
        latx, laty = -sin(h_mid), cos(h_mid)

        dx, dy = (x1 - x0), (y1 - y0)
        side_m = dx * latx + dy * laty
        side_ms = abs(side_m) / dt

        if side_ms > max_side_ms:
            max_side_ms = side_ms
        if side_ms > F.side_tol_ms:
            viol_frames.append(f)

        x0, y0, h0 = x1, y1, h1

    ok = (len(viol_frames) == 0)
    msg = "Max sideways {:.6f} m/s  Tol {:.6f} m/s  Violations {}".format(
        max_side_ms, F.side_tol_ms, len(viol_frames)
    )
    return {
        "ok": ok,
        "msg": msg,
        "violations": len(viol_frames),
        "violation_frames": viol_frames,
        "side_tol": F.side_tol_ms,
    }

# ---------- autocorrect ----------

def autocorrect(ctx):
    """
    Baseline autocorrect:
      - S: Bezier AUTO_CLAMPED on location keys
      - LIN: LINEAR on location keys
    Stores full snapshot for Revert. Params saved for bake.
    """
    scn = ctx.scene
    ch = scn.ra_props.chassis
    if not ch:
        return {"ok": False, "msg": "Assign the chassis."}

    act, fcs = _get_action_and_fcurves(ch)
    if act is None or not fcs:
        return {"ok": False, "msg": "Chassis has no keyed motion."}

    ns = bpy.app.driver_namespace
    if "TRA_AUTOBK" not in ns:
        ns["TRA_AUTOBK"] = _snapshot_fcurves(fcs)

    mode = scn.ra_feas.path_type
    if mode == 'S':
        _apply_s_curve(fcs, scn.ra_feas.tan_scale)
    else:
        _apply_linear(fcs)

    ns["TRA_AUTO"] = {
        "mode": mode,
        "tan_scale": scn.ra_feas.tan_scale,
        "rot_frac": scn.ra_feas.rot_frac,
    }
    return {"ok": True, "msg": "Autocorrect applied: {}".format('S-Curve' if mode == 'S' else 'Linear')}

def revert_autocorrect(ctx):
    scn = ctx.scene
    ch = scn.ra_props.chassis
    if not ch:
        return {"ok": False, "msg": "Assign the chassis."}

    ns = bpy.app.driver_namespace
    snap = ns.get("TRA_AUTOBK")
    if not snap:
        return {"ok": True, "msg": "Nothing to revert."}

    act, _ = _get_action_and_fcurves(ch)
    if act is None:
        return {"ok": False, "msg": "No action to restore."}

    ok = _restore_fcurves(ch, snap)
    if ok:
        ns.pop("TRA_AUTOBK", None)
        ns.pop("TRA_AUTO", None)
        return {"ok": True, "msg": "Autocorrect reverted."}
    return {"ok": False, "msg": "Revert failed."}

# Back-compat for ValidatePath
def check(ctx):
    return analyze(ctx)
