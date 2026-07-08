bl_info = {
    "name": "True RoboAnimator",
    "author": "Danyal S.",
    "version": (1, 4, 1),
    "blender": (4, 2, 0),
    "location": "3D Viewport > N-Panel > True RoboAnimator",
    "description": "Engineering-accurate animation toolkit for differential-drive and skid-steer robots.",
    "category": "Animation",
}

import bpy
import os
import json
import bisect
from math import pi, sin, cos, atan2, sqrt, floor
from mathutils import Vector


# ---------------------- Blender 4.4+ slotted-action compatibility ----------------------
def _bl_version():
    return bpy.app.version  # e.g. (5, 0, 0)


def _ensure_action_for(obj, action_name="Action"):
    """
    Ensure obj has animation_data + an assigned Action.
    On Blender 4.4+ a fresh action needs a slot bound to the object before
    fcurves.new() / keyframe_insert() will actually store data.
    Returns the assigned Action.
    """
    if obj.animation_data is None:
        obj.animation_data_create()
    ad = obj.animation_data
    if ad.action is None:
        ad.action = bpy.data.actions.new(name=action_name)
    act = ad.action

    # Slotted-action API (Blender 4.4+): make sure a slot is assigned.
    if hasattr(act, "slots"):
        try:
            slot = getattr(ad, "action_slot", None)
            if slot is None:
                if len(act.slots) == 0:
                    slot = act.slots.new(id_type='OBJECT', name=obj.name)
                else:
                    slot = act.slots[0]
                try:
                    ad.action_slot = slot
                except Exception:
                    pass
        except Exception:
            pass
    return act


def _iter_action_fcurves(action):
    """
    Iterate fcurves of an action across legacy and slotted representations.
    In Blender 4.4+ a slotted action can store its fcurves inside
    layers[i].strips[j].channelbags[k] instead of the top-level
    action.fcurves collection, so we walk both.
    """
    if action is None:
        return
    seen = set()

    fcs = getattr(action, "fcurves", None)
    if fcs is not None:
        for fc in fcs:
            key = (fc.data_path, fc.array_index)
            if key not in seen:
                seen.add(key)
                yield fc

    layers = getattr(action, "layers", None)
    if not layers:
        return
    for layer in layers:
        strips = getattr(layer, "strips", None) or []
        for strip in strips:
            cbgs = getattr(strip, "channelbags", None) or []
            for cbg in cbgs:
                cb_fcs = getattr(cbg, "fcurves", None)
                if not cb_fcs:
                    continue
                for fc in cb_fcs:
                    key = (fc.data_path, fc.array_index)
                    if key not in seen:
                        seen.add(key)
                        yield fc


def _find_channelbag_of(action, fc):
    layers = getattr(action, "layers", None) or []
    for layer in layers:
        strips = getattr(layer, "strips", None) or []
        for strip in strips:
            for cbg in getattr(strip, "channelbags", None) or []:
                for f in getattr(cbg, "fcurves", None) or []:
                    if f == fc:
                        return cbg
    return None


def _remove_action_fcurve(action, fc):
    # Try the legacy container first.
    fcs = getattr(action, "fcurves", None)
    if fcs is not None:
        try:
            fcs.remove(fc); return
        except Exception:
            pass
    # Fall back to slot channelbags.
    cbg = _find_channelbag_of(action, fc)
    if cbg is not None:
        try:
            cbg.fcurves.remove(fc)
        except Exception:
            pass


def _new_action_fcurve(action, data_path, index):
    # Reuse an existing fcurve if the compat iterator can find one.
    for fc in _iter_action_fcurves(action):
        if fc.data_path == data_path and fc.array_index == index:
            return fc

    fcs = getattr(action, "fcurves", None)
    if fcs is not None:
        try:
            return fcs.new(data_path=data_path, index=index)
        except Exception:
            pass

    layers = getattr(action, "layers", None)
    if layers:
        for layer in layers:
            for strip in getattr(layer, "strips", None) or []:
                for cbg in getattr(strip, "channelbags", None) or []:
                    try:
                        return cbg.fcurves.new(data_path=data_path, index=index)
                    except Exception:
                        continue
    return None


# ---------------------- constants & keys ----------------------
_AXIS_INDEX = {'X': 0, 'Y': 1, 'Z': 2}
_DEFKEY = "SEGWAY_RPM_V2_DATA"         # driver_namespace store key
_BACKUP_KEY = "RPM_AUTOCORRECT_BACKUP" # chassis keyframes backup (Text datablock)

def _driver_key():
    return _DEFKEY

# ---------------------- math helpers ----------------------
def _wrap(a):
    while a <= -pi: a += 2*pi
    while a >   pi: a -= 2*pi
    return a

def _unwrap(prev, cur):
    d = cur - prev
    while d >  pi: cur -= 2*pi; d = cur - prev
    while d < -pi: cur += 2*pi; d = cur - prev
    return cur

def _axis_unit(axis_char):
    return (1.0,0.0,0.0) if axis_char=='X' else (0.0,1.0,0.0) if axis_char=='Y' else (0.0,0.0,1.0)

def _auto_radius(obj, axis='Y'):
    d = obj.dimensions
    if axis=='Y': return 0.5*max(d.x,d.z)
    if axis=='X': return 0.5*max(d.y,d.z)
    return 0.5*max(d.x,d.y)

def _ensure_xyz_euler(o):
    if getattr(o,'rotation_mode','XYZ')!='XYZ': o.rotation_mode='XYZ'

def _ensure_quaternion(o):
    if getattr(o,'rotation_mode','XYZ')!='QUATERNION': o.rotation_mode='QUATERNION'

def _linerp(arr, idx_f):
    n=len(arr)
    if n==0: return 0.0
    if idx_f<=0: return arr[0]
    if idx_f>=n-1: return arr[-1]
    i0=int(floor(idx_f)); i1=i0+1; t=idx_f-i0
    return arr[i0]*(1.0-t)+arr[i1]*t

# ---------- Forward-axis ↔ yaw mapping ----------
def yaw_to_heading(P, yaw_world):
    fa=P.body_forward_axis
    if   fa=='+X': return _wrap(yaw_world)
    elif fa=='-X': return _wrap(yaw_world + pi)
    elif fa=='+Y': return _wrap(yaw_world + pi/2.0)
    else:          return _wrap(yaw_world - pi/2.0)  # -Y

def heading_to_yaw(P, heading):
    fa=P.body_forward_axis
    if   fa=='+X': return _wrap(heading)
    elif fa=='-X': return _wrap(heading - pi)
    elif fa=='+Y': return _wrap(heading - pi/2.0)
    else:          return _wrap(heading + pi/2.0)    # -Y

# ---------------------- Bezier tools for S-curve ----------------------
def _bezier_eval(P0,P1,P2,P3,t):
    mt=1.0-t
    x=(mt**3)*P0[0]+3*(mt**2)*t*P1[0]+3*mt*(t**2)*P2[0]+(t**3)*P3[0]
    y=(mt**3)*P0[1]+3*(mt**2)*t*P1[1]+3*mt*(t**2)*P2[1]+(t**3)*P3[1]
    dxdt=3*(mt**2)*(P1[0]-P0[0])+6*mt*t*(P2[0]-P1[0])+3*(t**2)*(P3[0]-P2[0])
    dydt=3*(mt**2)*(P1[1]-P0[1])+6*mt*t*(P2[1]-P1[1])+3*(t**2)*(P3[1]-P2[1])
    heading=atan2(dydt,dxdt)
    ddx=6*mt*(P2[0]-2*P1[0]+P0[0])+6*t*(P3[0]-2*P2[0]+P1[0])
    ddy=6*mt*(P2[1]-2*P1[1]+P0[1])+6*t*(P3[1]-2*P2[1]+P1[1])
    denom=(dxdt*dxdt+dydt*dydt)**1.5
    kappa=0.0 if denom<1e-9 else abs(dxdt*ddy - dydt*ddx)/denom
    return x,y,heading,kappa

def _ease_in_out_cubic(u):
    if u<0.5: return 4*u*u*u
    v=2*u-2; return 0.5*v*v*v+1.0

# ---------------------- NEW/CHANGED: true trapezoid mapping ----------------------
def _trapezoid_s(tau: float, f: float) -> float:
    """
    Proper trapezoid (position vs normalized time) with continuous speed.
    tau in [0,1], f = ramp_fraction in (0,0.5).
    Peak speed v_peak = 1/(1 - f) so total area (distance) is 1.
    Returns normalized arc-length s in [0,1].
    """
    if f <= 1e-12:
        return tau  # no ramps
    f = min(max(f, 0.0), 0.499999)
    v_peak = 1.0 / (1.0 - f)

    if tau <= f:
        # accel: v(τ)=v_peak*(τ/f), s=∫ v dτ = 0.5*v_peak/f * τ^2
        return 0.5 * v_peak * (tau*tau) / f

    if tau < 1.0 - f:
        # cruise: s(f) + v_peak*(τ−f)
        s_f = 0.5 * v_peak * f
        return s_f + v_peak * (tau - f)

    # decel: mirror; let u in [0,1]
    u = (tau - (1.0 - f)) / f
    s_start = 0.5 * v_peak * f + v_peak * (1.0 - 2.0*f)  # end of cruise
    return s_start + v_peak * f * (u - 0.5*u*u)
# --------------------------------------------------------------------------------

def _bezier_point_xy(P0,P1,P2,P3,t):
    mt=1.0-t
    x=(mt**3)*P0[0]+3*(mt**2)*t*P1[0]+3*mt*(t**2)*P2[0]+(t**3)*P3[0]
    y=(mt**3)*P0[1]+3*(mt**2)*t*P1[1]+3*mt*(t**2)*P2[1]+(t**3)*P3[1]
    return Vector((x,y))

def _bezier_tangent_xy(P0,P1,P2,P3,t):
    mt=1.0-t
    dxdt=3*(mt**2)*(P1[0]-P0[0])+6*mt*t*(P2[0]-P1[0])+3*(t**2)*(P3[0]-P2[0])
    dydt=3*(mt**2)*(P1[1]-P0[1])+6*mt*t*(P2[1]-P1[1])+3*(t**2)*(P3[1]-P2[1])
    return Vector((dxdt,dydt))

def _build_arc_lut_norm_total_xy(P0,P1,P2,P3,steps=128):
    pts=[_bezier_point_xy(P0,P1,P2,P3,i/steps) for i in range(steps+1)]
    s=[0.0]
    for i in range(1,len(pts)):
        s.append(s[-1]+(pts[i]-pts[i-1]).length)
    total=s[-1]
    if total<=1e-12: return [0.0 for _ in s], 0.0
    return [v/total for v in s], total

def _arc_to_t_from_lut(lut_norm, target_norm):
    lo,hi=0,len(lut_norm)-1
    if target_norm<=lut_norm[0]: return 0.0
    if target_norm>=lut_norm[-1]: return 1.0
    while lo+1<hi:
        mid=(lo+hi)//2
        if lut_norm[mid] < target_norm: lo=mid
        else: hi=mid
    span=lut_norm[hi]-lut_norm[lo]
    a=0.0 if span<=1e-12 else (target_norm-lut_norm[lo])/span
    return (lo+a)/(len(lut_norm)-1)

def _lerp(a,b,t): return a*(1.0-t)+b*t

# ---------------------- backup / restore chassis keys ----------------------
def _collect_fcurves(obj):
    ad = obj.animation_data
    if not (ad and ad.action):
        return []
    return [fc for fc in _iter_action_fcurves(ad.action)
            if fc.data_path in ("location", "rotation_euler")]

def _backup_chassis_keys(ch):
    data = {}
    for fc in _collect_fcurves(ch):
        key = f"{fc.data_path}[{fc.array_index}]"
        data[key] = [[float(kp.co[0]), float(kp.co[1])] for kp in fc.keyframe_points]
    key = f"{_BACKUP_KEY}_{bpy.context.scene.name}_{ch.name}"
    txt = bpy.data.texts.get(key) or bpy.data.texts.new(key)
    txt.clear()
    txt.write(json.dumps(data))
    return True

def _restore_chassis_keys(ch):
    key = f"{_BACKUP_KEY}_{bpy.context.scene.name}_{ch.name}"
    txt = bpy.data.texts.get(key)
    if not txt or len(txt.as_string()) == 0:
        return False
    try:
        data = json.loads(txt.as_string())
    except Exception:
        return False

    act = _ensure_action_for(ch, action_name="ChassisAction")
    for fc in list(_iter_action_fcurves(act)):
        if fc.data_path in ("location", "rotation_euler"):
            _remove_action_fcurve(act, fc)

    _ensure_xyz_euler(ch)
    for key, rows in data.items():
        if key.startswith("location["):
            path = "location"; idx = int(key.split("[")[1][0])
        elif key.startswith("rotation_euler["):
            path = "rotation_euler"; idx = int(key.split("[")[1][0])
        else:
            continue
        fc = _new_action_fcurve(act, path, idx)
        if fc is None:
            continue
        for fr, val in rows:
            fc.keyframe_points.insert(frame=fr, value=val, options={'FAST'})
    txt.clear()
    return True

# ---------------------- side helpers ----------------------
def _wheel_count(P):
    try:
        return int(P.num_wheels)
    except Exception:
        return 2


def _effective_track_width(P):
    """
    Return the track width used by every calculation.
    If auto_track_width is on and both wheel_l_01 and wheel_r_01 are set,
    it is the world distance between their origins. Otherwise the manual
    track_width value is used.
    """
    if getattr(P, "auto_track_width", True):
        wL = getattr(P, "wheel_l_01", None)
        wR = getattr(P, "wheel_r_01", None)
        if wL and wR:
            d = (wL.matrix_world.translation - wR.matrix_world.translation).length
            if d > 1e-5:
                return d
    return max(float(getattr(P, "track_width", 0.25)), 1e-5)


def _wheel_slots_for(P, side):
    """
    Return the ORDERED list of wheel-slot pointers for the given side
    ('L' or 'R'), respecting num_wheels and swap_lr.
    Empty slots and missing objects are filtered out later by _iter_side.
    """
    n = _wheel_count(P)
    if n == 2:
        left, right = [P.wheel_l_01], [P.wheel_r_01]
    elif n == 3:
        left, right = [P.wheel_l_01], [P.wheel_r_01]
    elif n == 4:
        left, right = [P.wheel_l_01, P.wheel_l_02], [P.wheel_r_01, P.wheel_r_02]
    elif n == 6:
        left  = [P.wheel_l_01, P.wheel_l_02, P.wheel_l_03]
        right = [P.wheel_r_01, P.wheel_r_02, P.wheel_r_03]
    else:
        left, right = [P.wheel_l_01], [P.wheel_r_01]

    if P.swap_lr:
        left, right = right, left
    return left if side == 'L' else right


def _iter_side(P, side):
    """Return the resolved list of wheel Objects on the given side."""
    if side == 'C':
        c = P.wheel_caster
        return [c] if (c and c.type in {'MESH', 'EMPTY'}) else []
    return [o for o in _wheel_slots_for(P, side)
            if o is not None and o.type in {'MESH', 'EMPTY'}]


def _iter_all_wheels(P):
    """L wheels + R wheels + caster (if configured), deduplicated."""
    seen, out = set(), []
    for side in ('L', 'R', 'C'):
        for o in _iter_side(P, side):
            k = id(o)
            if k not in seen:
                seen.add(k); out.append(o)
    return out

# ---------------------- body basis from heading + forward-axis ----------------------
def _body_basis_from_yaw(theta, forward_axis):
    c=cos(theta); s=sin(theta)
    if   forward_axis=='+Y': fwd=(-s, c)
    elif forward_axis=='-Y': fwd=( s,-c)
    elif forward_axis=='-X': fwd=(-c,-s)
    else:                    fwd=( c, s)  # +X
    lat=(-fwd[1], fwd[0])
    return fwd, lat

# ---------------------- validation (no sideways slip per frame) ----------------------
def analyze_motion(context):
    P=context.scene.sg_props; ch=P.chassis
    if not ch: raise RuntimeError("Assign the Chassis.")
    track = _effective_track_width(P)
    if track <= 0: raise RuntimeError("Track width must be > 0.")
    scn=context.scene; deps=context.evaluated_depsgraph_get()
    if scn.frame_end<=scn.frame_start: raise RuntimeError("Scene frame range invalid.")

    fps=scn.render.fps/scn.render.fps_base
    dt=1.0/float(fps)
    f0,f1=scn.frame_start,scn.frame_end
    scn.frame_set(f0); deps.update()
    prev_loc=ch.matrix_world.translation.copy()
    prev_yaw=ch.matrix_world.to_euler('XYZ').z

    violations=0; v_frames=[]
    for f in range(f0+1, f1+1):
        scn.frame_set(f); deps.update()
        loc=ch.matrix_world.translation
        yaw=_unwrap(prev_yaw, ch.matrix_world.to_euler('XYZ').z)
        yaw_mid=_wrap((prev_yaw+yaw)*0.5)
        dp_x=loc.x-prev_loc.x; dp_y=loc.y-prev_loc.y
        fwd,lat=_body_basis_from_yaw(yaw_mid, P.body_forward_axis)
        dx=dp_x*fwd[0] + dp_y*fwd[1]
        dy=dp_x*lat[0] + dp_y*lat[1]
        if abs(dy) > (P.side_tol*dt): violations+=1; v_frames.append(f)
        prev_loc=loc.copy(); prev_yaw=yaw
    return {'f0':f0,'fps':fps,'track':track,'violations':violations,'violation_frames':v_frames,'side_tol':P.side_tol,'dt':dt}

# ---------------------- gather keyed poses ----------------------
def _get_chassis_key_poses(P,ch):
    frames=set()
    ad=ch.animation_data
    if ad and ad.action:
        for fc in _iter_action_fcurves(ad.action):
            if fc.data_path in ("location","rotation_euler"):
                for kp in fc.keyframe_points:
                    frames.add(int(round(kp.co[0])))
    if not frames: return []
    scn=bpy.context.scene; deps=bpy.context.evaluated_depsgraph_get()
    out=[]
    for f in sorted(frames):
        scn.frame_set(f); deps.update()
        mw=ch.matrix_world; loc=mw.translation
        yaw=ch.matrix_world.to_euler('XYZ').z
        heading=yaw_to_heading(P, yaw)
        out.append((f, loc.x, loc.y, loc.z, yaw, heading))
    return out

# ---------------------- bake helper ----------------------
def _bake_chassis_frames_from_heading_samples(P, ch, samples):
    _ensure_xyz_euler(ch)
    scn = bpy.context.scene
    deps = bpy.context.evaluated_depsgraph_get()
    act = _ensure_action_for(ch, action_name="ChassisAction")
    for fc in list(_iter_action_fcurves(act)):
        if fc.data_path in ("location", "rotation_euler"):
            _remove_action_fcurve(act, fc)
    for f, x, y, z, h in samples:
        ch.location.x = x; ch.location.y = y; ch.location.z = z
        ch.rotation_euler[0] = 0.0; ch.rotation_euler[1] = 0.0
        ch.rotation_euler[2] = heading_to_yaw(P, h)
        ch.keyframe_insert("location", frame=f, index=-1)
        ch.keyframe_insert("rotation_euler", frame=f, index=-1)
    deps.update()
    scn.frame_set(scn.frame_start)
    deps.update()

# ---------------------- S-curve build + bake ----------------------
def _build_s_ease_curve_segment(poseA, poseB, tangent_start, tangent_end):
    """
    Cubic Bezier between two chassis waypoints. The start handle (P1)
    length is tangent_start * chord_distance, the end handle (P2) length
    is tangent_end * chord_distance. Both handles are clamped to at most
    0.49 * chord_distance so the curve never loops back on itself.
    """
    (x0, y0, h0) = poseA
    (x3, y3, h3) = poseB

    chord = atan2(y3 - y0, x3 - x0)
    def _align(h, ref):
        a = _wrap(h - ref)
        return h if abs(a) <= pi / 2 else _wrap(h + pi)
    h0 = _align(h0, chord)
    h3 = _align(h3, chord)

    P0 = (x0, y0); P3 = (x3, y3)
    dx = x3 - x0; dy = y3 - y0
    dist = max(1e-6, sqrt(dx * dx + dy * dy))
    cap = 0.49 * dist

    L_start = max(min(dist * tangent_start, cap), 1e-8)
    L_end   = max(min(dist * tangent_end,   cap), 1e-8)

    P1 = (x0 + L_start * cos(h0), y0 + L_start * sin(h0))
    P2 = (x3 - L_end   * cos(h3), y3 - L_end   * sin(h3))
    return P0, P1, P2, P3

def build_s_ease_curve_and_bake(context):
    P=context.scene.sg_props; ch=P.chassis
    if not ch: raise RuntimeError("Assign the Chassis.")
    scn=context.scene
    if not (ch.animation_data and ch.animation_data.action):
        raise RuntimeError("Chassis has no animation to autocorrect.")

    tangent_start = P.bezier_tangent_start
    tangent_end   = P.bezier_tangent_end

    profile=getattr(P,"speed_profile",'CONSTANT')
    f0,f1=scn.frame_start, scn.frame_end
    total_frames=max(1, f1-f0)

    ease_frames_tl=max(0, int(getattr(P,"timeline_ease_frames",15)))
    ease_frac_tl=(ease_frames_tl/total_frames) if total_frames>0 else 0.0

    seg_ease_frames=max(0, int(getattr(P,"segment_ease_frames",6)))

    # NEW/CHANGED: Constant profile ramp fraction
    const_ramp_frames=max(0, int(getattr(P,"constant_ramp_frames",0)))
    f_ramp = (const_ramp_frames/total_frames) if total_frames>0 else 0.0

    _backup_chassis_keys(ch)
    poses=_get_chassis_key_poses(P,ch)
    if len(poses)<2: raise RuntimeError("Need at least two keyed poses on the chassis (location/rotation).")

    segs=[]; total_len=0.0; spans=[]
    for i in range(len(poses)-1):
        fA,xA,yA,zA,_yA,hA=poses[i]
        fB,xB,yB,zB,_yB,hB=poses[i+1]
        spans.append(max(1, fB-fA))
        P0,P1,P2,P3=_build_s_ease_curve_segment((xA,yA,hA),(xB,yB,hB), tangent_start, tangent_end)
        lut_norm,L=_build_arc_lut_norm_total_xy(P0,P1,P2,P3,steps=128)
        if L<=1e-12: continue
        segs.append({"P0":P0,"P1":P1,"P2":P2,"P3":P3,"zA":zA,"zB":zB,"L":L,"lut":lut_norm})
        total_len+=L

    baked=[]; last_h=0.0
    cum_frames=[]; cum_len=[]; acc_f=0.0; acc_L=0.0
    for i,seg in enumerate(segs):
        acc_f+=spans[i]; acc_L+=seg["L"]
        cum_frames.append(acc_f); cum_len.append(acc_L)

    for f in range(f0, f1+1):
        tau=(f-f0)/max(1,total_frames)

        if profile=='CONSTANT':
            # NEW/CHANGED: proper trapezoid mapping (no speed jump)
            if f_ramp >= 0.5:
                raise RuntimeError("Ramp Frames too large: 2*R must be < total frames.")
            s_norm = _trapezoid_s(tau, f_ramp)
            s_target=s_norm*total_len
            idx=bisect.bisect_left(cum_len, s_target); idx=min(max(idx,0),len(segs)-1)
            s_acc=0.0 if idx==0 else cum_len[idx-1]
            seg=segs[idx]; frac=0.0 if seg["L"]<=1e-12 else (s_target-s_acc)/seg["L"]

        elif profile=='GLOBAL_EASE':
            tau2=_edge_ease_progress(tau, ease_frac_tl)
            s_target=tau2*total_len
            idx=bisect.bisect_left(cum_len, s_target); idx=min(max(idx,0),len(segs)-1)
            s_acc=0.0 if idx==0 else cum_len[idx-1]
            seg=segs[idx]; frac=0.0 if seg["L"]<=1e-12 else (s_target-s_acc)/seg["L"]

        else:  # PER_KEY_EASE
            tprime=tau*total_frames
            idx=bisect.bisect_left(cum_frames, tprime); idx=min(max(idx,0),len(segs)-1)
            fr_prev=0.0 if idx==0 else cum_frames[idx-1]
            span_f=max(1,int(round(cum_frames[idx]-fr_prev)))
            u=(tprime-fr_prev)/max(1e-9, span_f)
            fin=fout = seg_ease_frames / max(1.0, span_f)
            u=_edge_ease_progress_asym(u, fin, fout)
            seg=segs[idx]; frac=u

        t_param=_arc_to_t_from_lut(seg["lut"], frac)
        Pxy=_bezier_point_xy(seg["P0"],seg["P1"],seg["P2"],seg["P3"],t_param)
        Txy=_bezier_tangent_xy(seg["P0"],seg["P1"],seg["P2"],seg["P3"],t_param)
        h=atan2(Txy.y,Txy.x) if Txy.length>1e-9 else last_h
        z=_lerp(seg["zA"],seg["zB"],frac)
        baked.append((f, Pxy.x, Pxy.y, z, h)); last_h=h

    byf={fr:(fr,x,y,z,h) for (fr,x,y,z,h) in baked}
    out=[byf[k] for k in sorted(byf.keys())]
    _bake_chassis_frames_from_heading_samples(P,ch,out)
    return len(out)

# ---------------------- Linear (rotate→move→rotate) ----------------------
def _angle_lerp(prev,target,t):
    tgt=_unwrap(prev,target); return prev + t*(tgt-prev)

def build_linear_path_and_bake(context):
    """
    NEW/CHANGED for CONSTANT: apply the same global trapezoid arc-length mapping
    across translation segments so 'Constant' behaves consistently here too.
    Other profiles (GLOBAL_EASE, PER_KEY_EASE) unchanged.
    """
    P=context.scene.sg_props; ch=P.chassis
    if not ch: raise RuntimeError("Assign the Chassis.")
    scn=context.scene
    if not (ch.animation_data and ch.animation_data.action):
        raise RuntimeError("Chassis has no animation to autocorrect.")

    rot_frac=max(0.0, min(0.45, P.linear_rotation_fraction))
    profile=getattr(P,"speed_profile",'CONSTANT')

    seg_ease_frames=max(0, int(getattr(P,"segment_ease_frames",6)))
    const_ramp_frames=max(0, int(getattr(P,"constant_ramp_frames",0)))
    f0,f1=scn.frame_start, scn.frame_end
    total_frames=max(1, f1-f0)
    f_ramp = (const_ramp_frames/total_frames) if total_frames>0 else 0.0  # NEW/CHANGED

    _backup_chassis_keys(ch)
    poses=_get_chassis_key_poses(P,ch)
    if len(poses)<2: raise RuntimeError("Need at least two keyed poses on the chassis (location/rotation).")

    # Build segments with rotate-move-rotate behavior
    segs=[]; spans=[]; total_len=0.0
    for i in range(len(poses)-1):
        fA,xA,yA,zA,_yA,hA=poses[i]
        fB,xB,yB,zB,_yB,hB=poses[i+1]
        spans.append(max(1, fB-fA))
        dx=xB-xA; dy=yB-yA
        L=sqrt(dx*dx+dy*dy)
        
        if L<=1e-12:
            # Pure rotation segment
            segs.append({"mode":"rotonly","fA":fA,"fB":fB,"xA":xA,"yA":yA,"zA":zA,"xB":xB,"yB":yB,"zB":zB,"hA":hA,"hB":hB,"L":0.0})
        else:
            # Calculate direction to destination
            dir_to_dest = atan2(dy, dx)
            
            # Create three sub-segments: rotate to face destination, move straight, rotate to final orientation
            segs.append({
                "mode":"rotate_move_rotate",
                "fA":fA,"fB":fB,"xA":xA,"yA":yA,"zA":zA,"xB":xB,"yB":yB,"zB":zB,
                "hA":hA,"hB":hB,"L":L,
                "dir_to_dest":dir_to_dest,  # Direction robot should face to move to destination
                "final_heading":hB  # Final desired orientation
            })
            total_len+=L

    cum_len=[]; acc_L=0.0
    for s in segs:
        if s.get("L",0.0)>0.0:
            acc_L+=s["L"]
        cum_len.append(acc_L)

    baked=[]

    if profile=='CONSTANT':
        if f_ramp >= 0.5:
            raise RuntimeError("Ramp Frames too large: 2*R must be < total frames.")
        
        # For CONSTANT profile with rotate-move-rotate, we need to handle each segment's phases separately
        for i, seg in enumerate(segs):
            fA=seg["fA"]; fB=seg["fB"]; span=max(1, fB-fA)
            
            if seg["mode"] == "rotonly":
                # Pure rotation segment - just rotate in place
                hA=seg["hA"]; hB=seg["hB"]
                baked.append((fA, seg["xA"], seg["yA"], seg["zA"], hA))
                for s in range(1, span):
                    u=s/span; t=_ease_in_out_cubic(u)
                    h=_angle_lerp(hA, hB, t)
                    baked.append((fA+s, seg["xA"], seg["yA"], seg["zA"], h))
                baked.append((fB, seg["xB"], seg["yB"], seg["zB"], hB))
                
            elif seg["mode"] == "rotate_move_rotate":
                # Apply CONSTANT speed profile to the movement phase only
                hA=seg["hA"]; hB=seg["hB"]
                dir_to_dest=seg["dir_to_dest"]
                final_heading=seg["final_heading"]
                
                # Calculate frame distribution
                n1=int(round(span*rot_frac))  # Rotate to face destination
                n3=int(round(span*rot_frac))  # Rotate to final orientation  
                n2=span-n1-n3  # Move straight
                if n2<0:
                    shrink=-n2; s1=min(n1,(shrink+1)//2); s3=min(n3, shrink-s1)
                    n1-=s1; n3-=s3; n2=0

                baked.append((fA, seg["xA"], seg["yA"], seg["zA"], hA)); idx=0

                # Phase 1: Rotate in place to face destination (constant rotation speed)
                if n1>0:
                    for s in range(1,n1+1):
                        u=s/n1; t=_ease_in_out_cubic(u)
                        h=_angle_lerp(hA, dir_to_dest, t)
                        baked.append((fA+idx+s, seg["xA"], seg["yA"], seg["zA"], h))
                    idx+=n1

                # Phase 2: Move straight with CONSTANT speed profile
                if n2>0:
                    # Apply trapezoid speed profile to the movement phase
                    for s in range(1,n2+1):
                        tau=s/n2
                        s_norm=_trapezoid_s(tau, f_ramp)
                        u=s_norm  # Use the trapezoid profile for movement
                        x=seg["xA"]+u*(seg["xB"]-seg["xA"])
                        y=seg["yA"]+u*(seg["yB"]-seg["yA"])
                        z=seg["zA"]+u*(seg["zB"]-seg["zA"])
                        baked.append((fA+idx+s, x, y, z, dir_to_dest))
                    idx+=n2

                # Phase 3: Rotate in place to final orientation (constant rotation speed)
                if n3>0:
                    for s in range(1,n3+1):
                        u=s/n3; t=_ease_in_out_cubic(u)
                        h=_angle_lerp(dir_to_dest, final_heading, t)
                        baked.append((fA+idx+s, seg["xB"], seg["yB"], seg["zB"], h))
                    idx+=n3

                baked.append((fB, seg["xB"], seg["yB"], seg["zB"], final_heading))

    else:
        # NEW: Rotate-Move-Rotate behavior for realistic robot movement
        for i, seg in enumerate(segs):
            fA=seg["fA"]; fB=seg["fB"]; span=max(1, fB-fA)
            
            if seg["mode"] == "rotonly":
                # Pure rotation segment
                hA=seg["hA"]; hB=seg["hB"]
                baked.append((fA, seg["xA"], seg["yA"], seg["zA"], hA))
                for s in range(1, span):
                    u=s/span; t=_ease_in_out_cubic(u)
                    h=_angle_lerp(hA, hB, t)
                    baked.append((fA+s, seg["xA"], seg["yA"], seg["zA"], h))
                baked.append((fB, seg["xB"], seg["yB"], seg["zB"], hB))
                
            elif seg["mode"] == "rotate_move_rotate":
                # Rotate to face destination, move straight, rotate to final orientation
                hA=seg["hA"]; hB=seg["hB"]
                dir_to_dest=seg["dir_to_dest"]
                final_heading=seg["final_heading"]
                
                # Calculate frame distribution
                n1=int(round(span*rot_frac))  # Rotate to face destination
                n3=int(round(span*rot_frac))  # Rotate to final orientation  
                n2=span-n1-n3  # Move straight
                if n2<0:
                    shrink=-n2; s1=min(n1,(shrink+1)//2); s3=min(n3, shrink-s1)
                    n1-=s1; n3-=s3; n2=0

                baked.append((fA, seg["xA"], seg["yA"], seg["zA"], hA)); idx=0

                # Phase 1: Rotate in place to face destination
                if n1>0:
                    for s in range(1,n1+1):
                        u=s/n1; t=_ease_in_out_cubic(u)
                        h=_angle_lerp(hA, dir_to_dest, t)
                        baked.append((fA+idx+s, seg["xA"], seg["yA"], seg["zA"], h))
                    idx+=n1

                # Phase 2: Move straight to destination
                if n2>0:
                    if profile=='GLOBAL_EASE':
                        fin=fout=max(0.0, min(0.49, getattr(P,"timeline_ease_frames",15)/max(1.0,n2)))
                        for s in range(1,n2+1):
                            u=s/n2; u=_edge_ease_progress_asym(u, fin, fout)
                            x=seg["xA"]+u*(seg["xB"]-seg["xA"])
                            y=seg["yA"]+u*(seg["yB"]-seg["yA"])
                            z=seg["zA"]+u*(seg["zB"]-seg["zA"])
                            baked.append((fA+idx+s, x, y, z, dir_to_dest))
                    else:  # PER_KEY_EASE
                        fin=fout=seg_ease_frames/max(1.0,n2)
                        for s in range(1,n2+1):
                            u=s/n2; u=_edge_ease_progress_asym(u, fin, fout)
                            x=seg["xA"]+u*(seg["xB"]-seg["xA"])
                            y=seg["yA"]+u*(seg["yB"]-seg["yA"])
                            z=seg["zA"]+u*(seg["zB"]-seg["zA"])
                            baked.append((fA+idx+s, x, y, z, dir_to_dest))
                    idx+=n2

                # Phase 3: Rotate in place to final orientation
                if n3>0:
                    for s in range(1,n3+1):
                        u=s/n3; t=_ease_in_out_cubic(u)
                        h=_angle_lerp(dir_to_dest, final_heading, t)
                        baked.append((fA+idx+s, seg["xB"], seg["yB"], seg["zB"], h))
                    idx+=n3

                baked.append((fB, seg["xB"], seg["yB"], seg["zB"], final_heading))

    byf={fr:(fr,x,y,z,h) for (fr,x,y,z,h) in baked}
    out=[byf[k] for k in sorted(byf.keys())]
    _bake_chassis_frames_from_heading_samples(P,ch,out)
    return len(out)

# ---------------------- Edge ease tools (used by other profiles) ----------------------
def _edge_ease_progress(tau, ease_frac):
    """
    Symmetric edge ease on [0,1] with linear-ramp accel and decel.
    Position is C0 continuous at the joins tau=f and tau=1-f. Velocity has
    a small step at each join because the cruise velocity (1-f)/(1-2f) is
    higher than the ramp end velocity 1.0; for a C1 profile use
    _trapezoid_s instead.
    """
    f=max(0.0, min(0.49, ease_frac))
    if f<=1e-12: return tau
    if tau<=f: return (tau*tau)/(2.0*f)              # linear accel
    if tau>=1.0-f:
        u=1.0-tau; return 1.0 - (u*u)/(2.0*f)        # linear decel
    a=(1.0 - f)/(1.0 - 2.0*f)
    b=-f/(2.0*(1.0 - 2.0*f))
    return a*tau + b                                  # cruise

def _edge_ease_progress_asym(u, fin, fout):
    fin=max(0.0,min(0.49,fin)); fout=max(0.0,min(0.49,fout))
    mid0=fin; mid1=1.0-fout
    if u<=mid0 and fin>0.0:  return 0.5*(u*u)/max(1e-12,fin)
    if u>=mid1 and fout>0.0:
        v=1.0-u; return 1.0-0.5*(v*v)/max(1e-12,fout)
    m=mid1-mid0
    return 0.5*fin + (u-mid0)*(1.0-0.5*fin-0.5*fout)/max(1e-12,m)

# ---------------------- cache build for wheel drivers ----------------------
def _wheel_sign_auto(wheel_obj, wheel_axis_char, body_fwd_world):
    """
    Return +1 or -1 so that a positive theta rolls the vehicle forward.

    Derivation (rolling-without-slip constraint):
      Chassis moves forward at V along body_fwd. The wheel's ground-contact
      point at position r = -R*z_hat has velocity (in the wheel-center
      frame) v = omega x r = -R*omega*(A x z_hat), where A is the wheel's
      local rotation axis in world space. For no slip the contact point is
      stationary in the world frame, so v = -V * body_fwd.
      Solving: omega_scalar = V / (R * (A x z_hat) . body_fwd).
      Therefore sign(theta) = sign((A x up) . body_fwd) for V > 0.
    """
    if not wheel_obj:
        return 1.0
    m3 = wheel_obj.matrix_world.to_3x3()
    world_axis = m3 @ Vector(_axis_unit(wheel_axis_char))
    if world_axis.length < 1e-9:
        return 1.0
    world_axis.normalize()
    up = Vector((0.0, 0.0, 1.0))
    tangent = world_axis.cross(up)
    if tangent.length < 1e-9:
        return 1.0
    tangent.normalize()
    return 1.0 if tangent.dot(body_fwd_world) > 0.0 else -1.0


def _obj_rest_quat(obj):
    if not obj: return (1.0,0.0,0.0,0.0)
    if getattr(obj,'rotation_mode','XYZ')=='QUATERNION':
        q=(obj.rotation_quaternion[0], obj.rotation_quaternion[1], obj.rotation_quaternion[2], obj.rotation_quaternion[3])
    else:
        qq=obj.rotation_euler.to_quaternion(); q=(qq.w,qq.x,qq.y,qq.z)
    n=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]) or 1.0
    return (q[0]/n, q[1]/n, q[2]/n, q[3]/n)

def build_cache(context):
    """
    Validate, then integrate kinematics to produce per-frame wheel angles.
    Also enforces wheel RPM and RPM/s limits if set.
    """
    P=context.scene.sg_props; ch=P.chassis
    if not ch: raise RuntimeError("Assign the Chassis.")

    a=analyze_motion(context)
    if a['violations']>0:
        raise RuntimeError(f"Infeasible motion: {a['violations']} frame(s) exceed sideways tolerance > {a['side_tol']} "
                           f"(e.g., frames {a['violation_frames'][:10]}).")

    wL_list = _iter_side(P, 'L')
    wR_list = _iter_side(P, 'R')
    wC = P.wheel_caster if _wheel_count(P) == 3 else None
    wL0 = wL_list[0] if wL_list else None
    wR0 = wR_list[0] if wR_list else None
    if wL0 is None or wR0 is None:
        raise RuntimeError("Assign at least Left Wheel 01 and Right Wheel 01.")
    restL = _obj_rest_quat(wL0)
    restR = _obj_rest_quat(wR0)
    restC = _obj_rest_quat(wC) if wC else (1.0, 0.0, 0.0, 0.0)

    b = _effective_track_width(P)
    if P.auto_radius:
        w_ref = wL0 or wR0 or wC
        if not w_ref:
            raise RuntimeError("Assign at least one wheel object to enable auto radius detection.")
        radius = _auto_radius(w_ref, P.wheel_axis)
    else:
        radius = P.wheel_radius
    if radius <= 0:
        raise RuntimeError("Wheel radius must be > 0.")

    scn = context.scene
    deps = context.evaluated_depsgraph_get()
    fps = scn.render.fps / scn.render.fps_base
    dt = 1.0 / float(fps)
    f0 = scn.frame_start; f1 = scn.frame_end

    # Sample the chassis at the start frame so we can auto-derive per-wheel
    # rotation signs from geometry: whichever way the wheel's local rotation
    # axis actually points in world space, positive theta should mean forward
    # roll.
    scn.frame_set(f0); deps.update()
    initial_yaw = ch.matrix_world.to_euler('XYZ').z
    fwd_2d, _lat = _body_basis_from_yaw(initial_yaw, P.body_forward_axis)
    body_fwd_world = Vector((fwd_2d[0], fwd_2d[1], 0.0))
    if body_fwd_world.length < 1e-9:
        body_fwd_world = Vector((0.0, 1.0, 0.0))
    body_fwd_world.normalize()

    signL = _wheel_sign_auto(wL0, P.wheel_axis, body_fwd_world)
    signR = _wheel_sign_auto(wR0, P.wheel_axis, body_fwd_world)
    signC = _wheel_sign_auto(wC,  P.wheel_axis, body_fwd_world) if wC else 1.0

    thetaL = [0.0]; thetaR = [0.0]; thetaC = [0.0]
    pos_x = []; pos_y = []; yaw_z = []

    prev_loc = ch.matrix_world.translation.copy()
    prev_yaw = initial_yaw
    pos_x.append(prev_loc.x); pos_y.append(prev_loc.y); yaw_z.append(prev_yaw)
    tL = tR = tC = 0.0
    rpmL_list = [0.0]; rpmR_list = [0.0]; rpmC_list = [0.0]

    # In diff-drive kinematics the two driven sides share the same formula
    # regardless of how many wheels are mounted on each side; skid-steer 4/6
    # configurations just replicate the per-side wheel angle onto extra wheels
    # downstream. The caster (3-wheel only) rolls off dx alone.
    for f in range(f0 + 1, f1 + 1):
        scn.frame_set(f); deps.update()
        loc = ch.matrix_world.translation
        yaw = _unwrap(prev_yaw, ch.matrix_world.to_euler('XYZ').z)
        yaw_mid = _wrap((prev_yaw + yaw) * 0.5)

        dp_x = loc.x - prev_loc.x; dp_y = loc.y - prev_loc.y
        fwd, _lat = _body_basis_from_yaw(yaw_mid, P.body_forward_axis)
        dx = dp_x * fwd[0] + dp_y * fwd[1]

        dpsi = yaw - prev_yaw
        dsL = dx - 0.5 * b * dpsi
        dsR = dx + 0.5 * b * dpsi
        dthL = (dsL / radius) * signL
        dthR = (dsR / radius) * signR
        dthC = (dx  / radius) * signC  # caster: forward roll only

        tL += dthL; tR += dthR; tC += dthC
        thetaL.append(tL); thetaR.append(tR); thetaC.append(tC)
        pos_x.append(loc.x); pos_y.append(loc.y); yaw_z.append(yaw)

        rpmL = (dthL * fps) * 60.0 / (2.0 * pi)
        rpmR = (dthR * fps) * 60.0 / (2.0 * pi)
        rpmC = (dthC * fps) * 60.0 / (2.0 * pi)
        rpmL_list.append(rpmL); rpmR_list.append(rpmR); rpmC_list.append(rpmC)

        prev_loc = loc.copy(); prev_yaw = yaw

    # Only the driven wheels are checked against the safety limits;
    # a caster is passive and its RPM is a diagnostic, not a spec.
    max_rpm_limit    = float(getattr(P, "max_rpm", 0.0))
    max_arpm_s_limit = float(getattr(P, "max_ang_accel_rpm_s", 0.0))

    if max_rpm_limit > 0.0:
        over = []
        for i, (rL, rR) in enumerate(zip(rpmL_list, rpmR_list)):
            if abs(rL) > max_rpm_limit or abs(rR) > max_rpm_limit:
                over.append(f0 + i)
        if over:
            head = ", ".join(map(str, over[:12])) + (" ..." if len(over) > 12 else "")
            raise RuntimeError(f"Wheel RPM limit exceeded (>{max_rpm_limit:.1f} rpm) at frames: {head}")

    if max_arpm_s_limit > 0.0:
        over = []
        for i in range(1, len(rpmL_list)):
            aL = (rpmL_list[i] - rpmL_list[i - 1]) * fps
            aR = (rpmR_list[i] - rpmR_list[i - 1]) * fps
            if abs(aL) > max_arpm_s_limit or abs(aR) > max_arpm_s_limit:
                over.append(f0 + i)
        if over:
            head = ", ".join(map(str, over[:12])) + (" ..." if len(over) > 12 else "")
            raise RuntimeError(f"Wheel angular-accel limit exceeded (>{max_arpm_s_limit:.1f} rpm/s) at frames: {head}")

    data = {
        'f0': f0, 'fps': fps,
        # thetaX arrays are VISUAL rotations about each wheel's local axis
        # (what Blender drivers apply). To recover the PHYSICAL forward-roll
        # angle, multiply by the corresponding sign_* below (sign is +/-1).
        'thetaL': thetaL, 'thetaR': thetaR, 'thetaC': thetaC,
        'signL': signL, 'signR': signR, 'signC': signC,
        'x': pos_x, 'y': pos_y, 'yaw': yaw_z,
        'restL': restL, 'restR': restR, 'restC': restC,
        'radius': radius, 'track': b,
        'num_wheels': _wheel_count(P),
        'max_rpm_L': max(abs(r) for r in rpmL_list),
        'max_rpm_R': max(abs(r) for r in rpmR_list),
        'max_rpm_C': max(abs(r) for r in rpmC_list) if rpmC_list else 0.0,
        'violations': 0, 'violation_frames': [],
    }
    bpy.app.driver_namespace[_driver_key()] = data
    return data

# ---------------------- driver functions (for expressions) ----------------------
def _theta_array_for(d, side):
    if side == 'L': return d.get('thetaL', [0.0])
    if side == 'R': return d.get('thetaR', [0.0])
    return d.get('thetaC', [0.0])

def _rest_quat_for(d, side):
    if side == 'L': return d.get('restL', (1.0, 0.0, 0.0, 0.0))
    if side == 'R': return d.get('restR', (1.0, 0.0, 0.0, 0.0))
    return d.get('restC', (1.0, 0.0, 0.0, 0.0))

def sg_theta(side, frame):
    d = bpy.app.driver_namespace.get(_driver_key())
    if not d: return 0.0
    arr = _theta_array_for(d, side)
    i = int(frame - d['f0'])
    i = max(0, min(i, len(arr) - 1))
    return arr[i]

def sg_quat_comp(side, frame, comp_index, axis_char):
    d = bpy.app.driver_namespace.get(_driver_key())
    if not d:
        return (1.0, 0.0, 0.0, 0.0)[int(comp_index) % 4]
    th = sg_theta(side, frame)
    ux, uy, uz = _axis_unit(axis_char)
    h = 0.5 * th; s = sin(h); c = cos(h)
    q_spin = (c, ux*s, uy*s, uz*s)
    aw, ax, ay, az = _rest_quat_for(d, side)
    bw, bx, by, bz = q_spin
    q = (aw*bw - ax*bx - ay*by - az*bz,
         aw*bx + ax*bw + ay*bz - az*by,
         aw*by - ax*bz + ay*bw + az*bx,
         aw*bz + ax*by - ay*bx + az*bw)
    return q[int(comp_index) % 4]

def sg_quat_comp_obj(side, frame, comp_index, axis_char, rw, rx, ry, rz):
    d=bpy.app.driver_namespace.get(_driver_key())
    if not d:
        return (1.0,0.0,0.0,0.0)[int(comp_index)%4]
    th=sg_theta(side, frame)
    ux,uy,uz=_axis_unit(axis_char)
    h=0.5*th; s=sin(h); c=cos(h)
    q_spin=(c,ux*s,uy*s,uz*s)
    aw,ax,ay,az=(rw,rx,ry,rz)
    bw,bx,by,bz=q_spin
    q=(aw*bw-ax*bx-ay*by-az*bz,
       aw*bx+ax*bw+ay*bz-az*by,
       aw*by-ax*bz+ay*bw+az*bx,
       aw*bz+ax*by-ay*bx+az*bw)
    return q[int(comp_index)%4]

bpy.app.driver_namespace['sg_theta']=sg_theta
bpy.app.driver_namespace['sg_quat_comp']=sg_quat_comp
bpy.app.driver_namespace['sg_quat_comp_obj']=sg_quat_comp_obj

# ---------------------- viewport visualization ----------------------
# Thin GPU-drawn construction lines shown in every 3D View. Every time a
# related property changes, its update callback taggs the viewport for
# redraw, so the overlay follows the settings live.
import gpu
from gpu_extras.batch import batch_for_shader

_DRAW_HANDLE = None

# Colors are RGBA; alpha < 1 keeps them visibly "construction line" looking.
_CLR_FWD    = (0.30, 1.00, 0.35, 0.90)  # body forward axis (green)
_CLR_L      = (1.00, 0.35, 0.35, 0.90)  # left wheels (red)
_CLR_R      = (0.35, 0.55, 1.00, 0.90)  # right wheels (blue)
_CLR_C      = (1.00, 0.85, 0.20, 0.90)  # caster wheel (yellow)
_CLR_PATH_R = (1.00, 0.55, 0.10, 0.80)  # raw path (orange)
_CLR_PATH_S = (0.15, 0.85, 1.00, 0.90)  # solution path (cyan)


def _tag_viewport_redraw(_self=None, _ctx=None):
    """update= callback: mark every 3D viewport for a redraw."""
    try:
        wm = bpy.context.window_manager
        for w in wm.windows:
            for a in w.screen.areas:
                if a.type == 'VIEW_3D':
                    a.tag_redraw()
    except Exception:
        pass


def _apply_scale_rot_mesh(obj):
    """
    Bake scale and rotation into obj's mesh, preserving world location.
    Runs at the data level (no operator) so it is safe from a property
    update callback. Skips objects that would be surprising to modify:
    non-mesh, parented, shared-mesh, animated, or already identity.
    """
    if obj is None or obj.type != 'MESH':
        return False
    if obj.parent is not None:
        return False
    if obj.animation_data is not None and obj.animation_data.action is not None:
        return False
    mesh = obj.data
    if mesh is None:
        return False
    if getattr(mesh, "users", 1) > 1:
        return False

    scale_ok = all(abs(s - 1.0) < 1e-6 for s in obj.scale)
    rot_ok = all(abs(r) < 1e-6 for r in obj.rotation_euler) and \
             (obj.rotation_mode != 'QUATERNION' or (
                abs(obj.rotation_quaternion[0] - 1.0) < 1e-6 and
                all(abs(q) < 1e-6 for q in obj.rotation_quaternion[1:])))
    if scale_ok and rot_ok:
        return False

    if getattr(bpy.context, "mode", "OBJECT") != 'OBJECT':
        return False

    # matrix_basis = T * R * S in the object's own local frame; strip T,
    # transform the mesh by what remains, then reset R and S.
    mat = obj.matrix_basis.copy()
    mat.translation = (0.0, 0.0, 0.0)
    mesh.transform(mat)
    try:
        mesh.update()
    except Exception:
        pass

    obj.rotation_euler = (0.0, 0.0, 0.0)
    obj.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
    obj.scale = (1.0, 1.0, 1.0)
    return True


def _on_object_assigned(self, context):
    """PointerProperty update: viewport redraw + bake scale/rotation."""
    _tag_viewport_redraw(self, context)
    for prop in ("chassis",
                 "wheel_l_01", "wheel_l_02", "wheel_l_03",
                 "wheel_r_01", "wheel_r_02", "wheel_r_03",
                 "wheel_caster"):
        obj = getattr(self, prop, None)
        if obj is not None:
            _apply_scale_rot_mesh(obj)


def _on_body_forward_changed(self, context):
    """Auto-set wheel_axis perpendicular to body forward, plus viewport redraw."""
    _tag_viewport_redraw(self, context)
    fa = getattr(self, "body_forward_axis", "+Y")
    perp = 'Y' if fa in ('+X', '-X') else 'X'
    if self.wheel_axis != perp:
        self.wheel_axis = perp


def _axis_char_from_body_forward(fa):
    """Map '+Y'/'-X'/... to just the axis character."""
    return fa[-1] if fa and len(fa) >= 2 else 'Y'


def _iter_location_fcurves(ch):
    """Return (fc_x, fc_y, fc_z) triple or Nones for chassis location."""
    fcs = [None, None, None]
    if not ch or not ch.animation_data or not ch.animation_data.action:
        return fcs
    for fc in _iter_action_fcurves(ch.animation_data.action):
        if fc.data_path == "location" and 0 <= fc.array_index <= 2:
            fcs[fc.array_index] = fc
    return fcs


def _chassis_keyframe_positions(ch):
    """List of (x, y, z) at every location keyframe on the chassis."""
    fcs = _iter_location_fcurves(ch)
    if not any(fcs):
        return []
    frames = set()
    for fc in fcs:
        if fc:
            for kp in fc.keyframe_points:
                frames.add(int(round(kp.co[0])))
    out = []
    for f in sorted(frames):
        x = fcs[0].evaluate(f) if fcs[0] else 0.0
        y = fcs[1].evaluate(f) if fcs[1] else 0.0
        z = fcs[2].evaluate(f) if fcs[2] else 0.0
        out.append((x, y, z))
    return out


def _chassis_backup_positions(ch):
    """List of (x, y, z) waypoints stored by the autocorrect backup, if any."""
    if not ch:
        return []
    key = f"{_BACKUP_KEY}_{bpy.context.scene.name}_{ch.name}"
    txt = bpy.data.texts.get(key)
    if not txt or not txt.as_string():
        return []
    try:
        data = json.loads(txt.as_string())
    except Exception:
        return []
    channels = {0: {}, 1: {}, 2: {}}
    frames = set()
    for k, rows in data.items():
        if k.startswith("location["):
            try:
                idx = int(k.split("[")[1][0])
            except Exception:
                continue
            if idx not in channels:
                continue
            for fr, val in rows:
                channels[idx][int(round(fr))] = float(val)
                frames.add(int(round(fr)))
    out = []
    last = [0.0, 0.0, 0.0]
    for f in sorted(frames):
        for i in range(3):
            if f in channels[i]:
                last[i] = channels[i][f]
        out.append((last[0], last[1], last[2]))
    return out


def _chassis_smooth_path(ch, f0, f1, step=1):
    """Dense sample of the current chassis fcurve interpolation."""
    fcs = _iter_location_fcurves(ch)
    if not any(fcs):
        return []
    pts = []
    for f in range(int(f0), int(f1) + 1, step):
        x = fcs[0].evaluate(f) if fcs[0] else 0.0
        y = fcs[1].evaluate(f) if fcs[1] else 0.0
        z = fcs[2].evaluate(f) if fcs[2] else 0.0
        pts.append((x, y, z))
    return pts


def _draw_lines(pairs, color, width=1.5):
    """Draw a list of ((x,y,z),(x,y,z)) segments in one batch."""
    if not pairs:
        return
    verts = []
    for a, b in pairs:
        verts.append(tuple(a)); verts.append(tuple(b))
    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    batch = batch_for_shader(shader, 'LINES', {"pos": verts})
    prev_lw = 1.0
    try:
        gpu.state.line_width_set(width)
        shader.bind()
        shader.uniform_float("color", color)
        batch.draw(shader)
    finally:
        gpu.state.line_width_set(prev_lw)


def _draw_polyline(pts, color, width=1.5):
    if len(pts) < 2:
        return
    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": [tuple(p) for p in pts]})
    try:
        gpu.state.line_width_set(width)
        shader.bind()
        shader.uniform_float("color", color)
        batch.draw(shader)
    finally:
        gpu.state.line_width_set(1.0)


def _draw_points(pts, color, size=7.0):
    if not pts:
        return
    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    batch = batch_for_shader(shader, 'POINTS', {"pos": [tuple(p) for p in pts]})
    try:
        gpu.state.point_size_set(size)
        shader.bind()
        shader.uniform_float("color", color)
        batch.draw(shader)
    finally:
        gpu.state.point_size_set(1.0)


def _forward_axis_world(ch, body_forward_axis, length):
    """Return (origin, tip, tick_a, tick_b) in world space for the forward-axis arrow."""
    axis_char = _axis_char_from_body_forward(body_forward_axis)
    sgn = -1.0 if body_forward_axis.startswith('-') else 1.0
    lv = _axis_unit(axis_char)
    local = Vector((lv[0] * sgn, lv[1] * sgn, lv[2] * sgn))
    m3 = ch.matrix_world.to_3x3()
    world = m3 @ local
    if world.length < 1e-9:
        world = Vector((0.0, 1.0, 0.0))
    world.normalize()
    origin = ch.matrix_world.translation.copy()
    tip = origin + world * length
    # Arrow-head perpendicular in the local XY of the arrow.
    up = Vector((0.0, 0.0, 1.0))
    perp = world.cross(up)
    if perp.length < 1e-6:
        perp = Vector((1.0, 0.0, 0.0))
    perp.normalize()
    head = length * 0.18
    a = tip - world * head + perp * head * 0.55
    b = tip - world * head - perp * head * 0.55
    return origin, tip, a, b


def _wheel_axis_segments(obj, wheel_axis_char, side):
    """
    Build the segments for one wheel's rotation-axis gizmo:
      - a line along the rotation axis, total length ~= 2 * tire width
      - a small perpendicular tick at the (+) end of the axis, indicating
        the positive-theta side by the right-hand rule
    """
    if not obj:
        return []
    m3 = obj.matrix_world.to_3x3()
    lv = _axis_unit(wheel_axis_char)
    world_axis = m3 @ Vector(lv)
    if world_axis.length < 1e-9:
        world_axis = Vector((0.0, 1.0, 0.0))
    world_axis.normalize()

    # Estimate tire width as the smallest bbox dimension of the wheel.
    # For a typical cylindrical wheel this is the width along the axle.
    dims = obj.dimensions
    tire_w = max(min(dims.x, dims.y, dims.z), 0.02)
    half = tire_w  # so total axis line = 2 * tire_w
    center = obj.matrix_world.translation.copy()
    a = center - world_axis * half
    b = center + world_axis * half

    up = Vector((0.0, 0.0, 1.0))
    tick = world_axis.cross(up)
    if tick.length < 1e-6:
        tick = Vector((1.0, 0.0, 0.0))
    tick.normalize()
    tick_len = tire_w * 0.4
    t1 = b + tick * tick_len
    t2 = b - tick * tick_len

    return [(a, b), (b, t1), (b, t2)]


def _draw_body_forward(P):
    ch = P.chassis
    if not ch:
        return
    length = max(_effective_track_width(P) * 2.5, 0.5)
    origin, tip, a, b = _forward_axis_world(ch, P.body_forward_axis, length)
    _draw_lines([(origin, tip), (tip, a), (tip, b)], _CLR_FWD, width=2.0)


def _draw_all_wheels(P):
    wa = P.wheel_axis
    segs_L = []
    for obj in _iter_side(P, 'L'):
        segs_L.extend(_wheel_axis_segments(obj, wa, 'L'))
    segs_R = []
    for obj in _iter_side(P, 'R'):
        segs_R.extend(_wheel_axis_segments(obj, wa, 'R'))
    segs_C = []
    for obj in _iter_side(P, 'C'):
        segs_C.extend(_wheel_axis_segments(obj, wa, 'C'))

    _draw_lines(segs_L, _CLR_L, width=1.8)
    _draw_lines(segs_R, _CLR_R, width=1.8)
    _draw_lines(segs_C, _CLR_C, width=1.8)


def _preview_key_poses(P, ch):
    """
    Waypoints (frame, x, y, z, yaw, heading) read directly from the chassis
    fcurves via fc.evaluate. Cheap enough for viewport redraws. Supports
    Euler and Quaternion rotation.
    """
    if not ch or not ch.animation_data or not ch.animation_data.action:
        return []
    act = ch.animation_data.action
    loc = [None, None, None]
    reul = [None, None, None]
    quat = [None, None, None, None]
    for fc in _iter_action_fcurves(act):
        if fc.data_path == "location" and 0 <= fc.array_index <= 2:
            loc[fc.array_index] = fc
        elif fc.data_path == "rotation_euler" and 0 <= fc.array_index <= 2:
            reul[fc.array_index] = fc
        elif fc.data_path == "rotation_quaternion" and 0 <= fc.array_index <= 3:
            quat[fc.array_index] = fc
    if not any(loc):
        return []
    frames = set()
    for fc in loc + reul + list(quat):
        if fc:
            for kp in fc.keyframe_points:
                frames.add(int(round(kp.co[0])))
    out = []
    for f in sorted(frames):
        x = loc[0].evaluate(f) if loc[0] else 0.0
        y = loc[1].evaluate(f) if loc[1] else 0.0
        z = loc[2].evaluate(f) if loc[2] else 0.0
        if reul[2] is not None:
            yaw = reul[2].evaluate(f)
        elif all(quat):
            qw = quat[0].evaluate(f); qx = quat[1].evaluate(f)
            qy = quat[2].evaluate(f); qz = quat[3].evaluate(f)
            yaw = atan2(2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz))
        else:
            yaw = 0.0
        heading = yaw_to_heading(P, yaw)
        out.append((f, x, y, z, yaw, heading))
    return out


def _preview_backup_poses(P, ch):
    """(frame, x, y, z, yaw, heading) reconstructed from the autocorrect backup Text."""
    if not ch:
        return []
    key = f"{_BACKUP_KEY}_{bpy.context.scene.name}_{ch.name}"
    txt = bpy.data.texts.get(key)
    if not txt or not txt.as_string():
        return []
    try:
        data = json.loads(txt.as_string())
    except Exception:
        return []
    loc_ch = {0: {}, 1: {}, 2: {}}
    yaw_map = {}
    frames = set()
    for k, rows in data.items():
        if k.startswith("location["):
            try:
                idx = int(k.split("[")[1][0])
            except Exception:
                continue
            if idx not in loc_ch:
                continue
            for fr, val in rows:
                loc_ch[idx][int(round(fr))] = float(val)
                frames.add(int(round(fr)))
        elif k == "rotation_euler[2]":
            for fr, val in rows:
                yaw_map[int(round(fr))] = float(val)
                frames.add(int(round(fr)))
    out = []
    lx = ly = lz = 0.0
    lyaw = 0.0
    for f in sorted(frames):
        if f in loc_ch[0]: lx = loc_ch[0][f]
        if f in loc_ch[1]: ly = loc_ch[1][f]
        if f in loc_ch[2]: lz = loc_ch[2][f]
        if f in yaw_map:   lyaw = yaw_map[f]
        out.append((f, lx, ly, lz, lyaw, yaw_to_heading(P, lyaw)))
    return out


def _preview_sease_geometry(P, poses, steps_per_seg=32):
    """
    Sample the S-Ease Bezier curve geometry (no timing) so the preview
    reflects Tangent Scale Start / End and body forward immediately.
    """
    tangent_start = P.bezier_tangent_start
    tangent_end   = P.bezier_tangent_end
    pts = []
    for i in range(len(poses) - 1):
        _fA, xA, yA, zA, _yA, hA = poses[i]
        _fB, xB, yB, zB, _yB, hB = poses[i + 1]
        P0, P1, P2, P3 = _build_s_ease_curve_segment(
            (xA, yA, hA), (xB, yB, hB), tangent_start, tangent_end)
        for s in range(steps_per_seg + 1):
            t = s / steps_per_seg
            pt = _bezier_point_xy(P0, P1, P2, P3, t)
            z = _lerp(zA, zB, t)
            # Skip duplicate junction point when we start the next segment.
            if pts and s == 0:
                continue
            pts.append((pt.x, pt.y, z))
    return pts


def _preview_linear_geometry(_P, poses):
    """Linear autocorrect is a straight polyline through the waypoints."""
    return [(x, y, z) for (_f, x, y, z, _yaw, _h) in poses]


def _preview_solution_path(P, ch):
    """
    Solution path preview:
    - autocorrect OFF: sample the CURRENT chassis fcurves (baked animation).
    - autocorrect SEASE: geometry of the S-Ease Bezier segments.
    - autocorrect LINEAR: straight polyline through waypoints.
    Waypoints come from the autocorrect backup when present so the preview
    tracks the user's original intent even after a bake.
    """
    if not ch:
        return []
    mode = P.autocorrect_mode
    if mode == 'OFF':
        scn = bpy.context.scene
        return _chassis_smooth_path(ch, int(scn.frame_start), int(scn.frame_end), step=1)
    poses = _preview_backup_poses(P, ch) or _preview_key_poses(P, ch)
    if len(poses) < 2:
        return []
    if mode == 'SEASE':
        return _preview_sease_geometry(P, poses)
    if mode == 'LINEAR':
        return _preview_linear_geometry(P, poses)
    return []


def _draw_paths(P):
    ch = P.chassis
    if not ch:
        return

    if P.vis_show_raw_path:
        backup = _chassis_backup_positions(ch)
        raw_pts = backup or _chassis_keyframe_positions(ch)
        if raw_pts:
            _draw_polyline(raw_pts, _CLR_PATH_R, width=1.2)
            _draw_points(raw_pts, _CLR_PATH_R, size=8.0)

    if P.vis_show_solution_path:
        sol = _preview_solution_path(P, ch)
        if sol:
            _draw_polyline(sol, _CLR_PATH_S, width=2.0)


def _draw_handler():
    try:
        P = bpy.context.scene.sg_props
    except Exception:
        return
    if not getattr(P, "vis_enabled", True):
        return

    # Preserve GPU state so we don't leak into other overlays.
    prev_blend = gpu.state.blend_get()
    prev_depth = gpu.state.depth_test_get()
    gpu.state.blend_set('ALPHA')
    gpu.state.depth_test_set('NONE')  # construction lines show through geometry

    try:
        if P.vis_show_forward:
            _draw_body_forward(P)
        if P.vis_show_wheels:
            _draw_all_wheels(P)
        if P.vis_show_raw_path or P.vis_show_solution_path:
            _draw_paths(P)
    finally:
        gpu.state.blend_set(prev_blend)
        gpu.state.depth_test_set(prev_depth)


def _register_draw_handler():
    global _DRAW_HANDLE
    if _DRAW_HANDLE is None:
        _DRAW_HANDLE = bpy.types.SpaceView3D.draw_handler_add(
            _draw_handler, (), 'WINDOW', 'POST_VIEW')


def _unregister_draw_handler():
    global _DRAW_HANDLE
    if _DRAW_HANDLE is not None:
        try:
            bpy.types.SpaceView3D.draw_handler_remove(_DRAW_HANDLE, 'WINDOW')
        except Exception:
            pass
        _DRAW_HANDLE = None


# ---------------------- properties ----------------------
class SG_Props(bpy.types.PropertyGroup):
    # Selection
    chassis: bpy.props.PointerProperty(name="Chassis (animated)", type=bpy.types.Object, update=_on_object_assigned)

    num_wheels: bpy.props.EnumProperty(
        name="Wheel Configuration",
        description="Number and layout of the driven / passive wheels",
        items=[
            ('2', "2 (Diff Drive)",       "1 driven wheel per side"),
            ('3', "3 (Diff + Caster)",    "1 driven wheel per side plus a passive caster / omni wheel"),
            ('4', "4 (Skid Steer)",       "2 driven wheels per side, same speed on each side"),
            ('6', "6 (Rocker / Rover)",   "3 driven wheels per side, same speed on each side"),
        ],
        default='2',
        update=_tag_viewport_redraw,
    )

    # Individual wheel object slots. Only the ones needed by num_wheels are
    # shown in the UI; the rest are ignored.
    wheel_l_01: bpy.props.PointerProperty(name="Left Wheel 01",  type=bpy.types.Object, update=_on_object_assigned)
    wheel_l_02: bpy.props.PointerProperty(name="Left Wheel 02",  type=bpy.types.Object, update=_on_object_assigned)
    wheel_l_03: bpy.props.PointerProperty(name="Left Wheel 03",  type=bpy.types.Object, update=_on_object_assigned)
    wheel_r_01: bpy.props.PointerProperty(name="Right Wheel 01", type=bpy.types.Object, update=_on_object_assigned)
    wheel_r_02: bpy.props.PointerProperty(name="Right Wheel 02", type=bpy.types.Object, update=_on_object_assigned)
    wheel_r_03: bpy.props.PointerProperty(name="Right Wheel 03", type=bpy.types.Object, update=_on_object_assigned)
    wheel_caster: bpy.props.PointerProperty(name="Caster Wheel", type=bpy.types.Object, update=_on_object_assigned)

    swap_lr: bpy.props.BoolProperty(
        name="Swap L/R Sides",
        description="Swap the roles of the left and right wheel slots without re-assigning them",
        default=False,
        update=_tag_viewport_redraw,
    )

    # Geometry / wheels
    auto_track_width: bpy.props.BoolProperty(
        name="Auto Track Width",
        description="Measure track width as the world distance between Left Wheel 01 and Right Wheel 01",
        default=True, update=_tag_viewport_redraw,
    )
    track_width: bpy.props.FloatProperty(
        name="Track Width (m)",
        description="Center-to-center distance between the left and right wheels along the axle",
        default=0.25, min=1e-5, precision=5, update=_tag_viewport_redraw,
    )
    auto_radius: bpy.props.BoolProperty(name="Auto Wheel Radius", default=True)
    wheel_radius: bpy.props.FloatProperty(name="Wheel Radius (m)", default=0.06, min=1e-5, precision=5)
    wheel_axis: bpy.props.EnumProperty(
        name="Wheel Rotation Axis",
        description="Local axis the wheels spin around. Auto-set to be perpendicular to Body Forward Axis",
        items=[('X','X',''),('Y','Y',''),('Z','Z','')],
        default='X', update=_tag_viewport_redraw,
    )
    rotation_mode: bpy.props.EnumProperty(name="Rotation Mode", items=[('EULER','Euler',''),('QUAT','Quaternion','')], default='EULER')

    # Feasibility / Autocorrect
    body_forward_axis: bpy.props.EnumProperty(
        name="Body Forward Axis",
        items=[('+X','Local +X',''), ('-X','Local -X',''), ('+Y','Local +Y',''), ('-Y','Local -Y','')],
        default='+Y',
        update=_on_body_forward_changed,
    )
    side_tol: bpy.props.FloatProperty(name="Sideways Tolerance (m/s)", default=0.02, min=0.0, precision=6)

    autocorrect_mode: bpy.props.EnumProperty(
        name="Autocorrect Mode",
        items=[
            ('OFF','Off',''),
            ('SEASE','Smooth Curve (S-Ease)','Smooth Bezier segments with curvature clamp'),
            ('LINEAR','Linear (Rotate-Move-Rotate)','Rotate to face, move straight, rotate to final'),
        ],
        default='SEASE',
        update=_tag_viewport_redraw,
    )
    bezier_tangent_start: bpy.props.FloatProperty(
        name="Tangent Scale Start",
        description="Length of the start-handle (P1) of each Bezier segment as a fraction of the pose distance. Larger = the curve leaves the start waypoint with a bigger swing",
        default=0.35, min=0.0, max=0.49, precision=3,
        update=_tag_viewport_redraw,
    )
    bezier_tangent_end: bpy.props.FloatProperty(
        name="Tangent Scale End",
        description="Length of the end-handle (P2) of each Bezier segment as a fraction of the pose distance. Larger = the curve approaches the end waypoint with a bigger swing",
        default=0.35, min=0.0, max=0.49, precision=3,
        update=_tag_viewport_redraw,
    )
    linear_rotation_fraction: bpy.props.FloatProperty(
        name="Rotation Fraction",
        description="Per segment, fraction of frames used for the initial and final rotations (each)",
        default=0.25, min=0.0, max=0.45, precision=3,
        update=_tag_viewport_redraw,
    )

    # Speed profiles
    speed_profile: bpy.props.EnumProperty(
        name="Speed Profile (Timing)",
        items=[
            ('CONSTANT',     "Constant (Uniform+Ramps)",    "Constant speed with ramps (linear accel/decel)"),
            ('GLOBAL_EASE',  "Ease - Whole Timeline",       "Slow-in/fast/slow-out across the entire timeline"),
            ('PER_KEY_EASE', "Ease - Per Keyframe Segment", "Each keyframe-to-keyframe segment eases (single slider)"),
        ],
        default='CONSTANT',
    )
    # NEW/CHANGED: ramps for CONSTANT (trapezoid)
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
    # single control instead of in/out
    segment_ease_frames: bpy.props.IntProperty(
        name="Ease Frames (Per Segment)",
        description="Symmetric ease-in/out within each keyframe segment.",
        min=0, max=400, default=6,
    )

    # CSV / units
    csv_path: bpy.props.StringProperty(name="CSV File", default="//robot_anim.csv", subtype='FILE_PATH')
    sample_mode: bpy.props.EnumProperty(name="Sampling", items=[('FRAME','Every Frame (scene FPS)',''), ('FIXED','Fixed Rate (Hz)','')], default='FRAME')
    fixed_rate: bpy.props.IntProperty(name="Rate (Hz)", min=1, max=5000, default=100)
    angle_unit: bpy.props.EnumProperty(name="Angle Unit", items=[('RAD','radians',''),('DEG','degrees','')], default='RAD')
    angrate_unit: bpy.props.EnumProperty(name="Angular Rate", items=[('RPM','rpm',''),('RPS','rps',''),('DEGS','deg/s','')], default='RPM')
    length_unit: bpy.props.EnumProperty(name="Length Unit", items=[('M','meters',''),('CM','centimeters','')], default='M')

    # Safety limits
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

    # CSV Import
    csv_import_path: bpy.props.StringProperty(name="CSV Import File", default="//import_anim.csv", subtype='FILE_PATH')
    csv_import_skip_rows: bpy.props.IntProperty(name="Skip Rows from Top", default=2, min=0, max=100, description="Number of header/metadata rows to skip")

    # UI foldouts (kept in scene so they persist per-file)
    show_instructions:  bpy.props.BoolProperty(name="Show Instructions",       default=False)
    show_viewport:      bpy.props.BoolProperty(name="Show Viewport Helpers",   default=True)
    show_setup:         bpy.props.BoolProperty(name="Show Rig Setup",          default=True)
    show_calibration:   bpy.props.BoolProperty(name="Show Calibration",        default=False)
    show_motion:        bpy.props.BoolProperty(name="Show Motion Path",        default=True)
    show_drivers:       bpy.props.BoolProperty(name="Show Wheel Drivers",      default=True)
    show_io:            bpy.props.BoolProperty(name="Show Import / Export",    default=False)

    io_mode: bpy.props.EnumProperty(
        name="Operation",
        items=[
            ('IMPORT', "Import CSV",         "Read a chassis + wheel trajectory back from CSV"),
            ('KEYS',   "Export Keyframes",   "Snapshot keyframe rows to CSV or JSON"),
            ('CSV',    "Engineering CSV",    "Continuous time-series CSV at scene or fixed rate"),
        ],
        default='CSV',
    )

    # Viewport visualization toggles
    vis_enabled: bpy.props.BoolProperty(
        name="Enable Overlay",
        description="Master toggle for all viewport construction lines drawn by this add-on",
        default=True, update=_tag_viewport_redraw,
    )
    vis_show_forward: bpy.props.BoolProperty(
        name="Body Forward Axis",
        description="Green arrow from the chassis along the chosen Body Forward Axis",
        default=True, update=_tag_viewport_redraw,
    )
    vis_show_wheels: bpy.props.BoolProperty(
        name="Wheel Rotation Axes",
        description="Colored axis line through each configured wheel (L red, R blue, caster yellow)",
        default=True, update=_tag_viewport_redraw,
    )
    vis_show_raw_path: bpy.props.BoolProperty(
        name="Raw Path",
        description="Orange polyline through the chassis keyframes (uses autocorrect backup when present)",
        default=True, update=_tag_viewport_redraw,
    )
    vis_show_solution_path: bpy.props.BoolProperty(
        name="Solution Path",
        description="Cyan curve sampling the current chassis animation across every frame",
        default=True, update=_tag_viewport_redraw,
    )

    # Keyframe export
    other_export_path: bpy.props.StringProperty(name="Anim Data File", default="//anim_keyframes.csv", subtype='FILE_PATH')
    other_export_format: bpy.props.EnumProperty(name="Format", items=[('CSV','CSV',''),('JSON','JSON','')], default='CSV')
    other_angle_unit: bpy.props.EnumProperty(name="Angle Unit (Anim)", items=[('RAD','radians',''),('DEG','degrees','')], default='RAD')

# ---------------------- operators ----------------------
class SG_OT_ValidateMotion(bpy.types.Operator):
    bl_idname="segway.validate_motion"; bl_label="Validate Motion"
    bl_description="Check current chassis animation for nonholonomic feasibility (mid-step heading)"
    def execute(self, context):
        try: a=analyze_motion(context)
        except Exception as e: self.report({'ERROR'}, str(e)); return {'CANCELLED'}
        if a['violations']>0:
            frames=a['violation_frames']; head=", ".join(str(f) for f in frames[:12]) + (" …" if len(frames)>12 else "")
            self.report({'ERROR'}, f"This won't work: {a['violations']} step(s) exceed sideways tolerance > {a['side_tol']} (frames {head}).")
            return {'CANCELLED'}
        self.report({'INFO'}, "Motion is feasible (no slip violations)."); return {'FINISHED'}

class SG_OT_AutocorrectBake(bpy.types.Operator):
    bl_idname="segway.autocorrect_bake"; bl_label="Autocorrect & Bake"
    bl_description="Bake using the selected Autocorrect Mode (S-Ease or Linear) with the chosen Speed Profile"
    def execute(self, context):
        P=context.scene.sg_props; mode=P.autocorrect_mode
        try:
            if   mode=='SEASE':  n=build_s_ease_curve_and_bake(context)
            elif mode=='LINEAR': n=build_linear_path_and_bake(context)
            else: self.report({'ERROR'},"Set Autocorrect Mode to S-Ease or Linear."); return {'CANCELLED'}
        except Exception as e:
            self.report({'ERROR'}, f"Autocorrect failed: {e}"); return {'CANCELLED'}
        self.report({'INFO'}, f"Autocorrect baked {n} frames. Re-run Validate Motion."); return {'FINISHED'}

class SG_OT_AutocorrectSEase(bpy.types.Operator):
    bl_idname="segway.autocorrect_sease"; bl_label="Autocorrect & Bake (Smooth S-Ease)"
    def execute(self, context):
        P=context.scene.sg_props
        if P.autocorrect_mode!='SEASE': self.report({'ERROR'},"Set Autocorrect Mode to 'Smooth Curve (S-Ease)'."); return {'CANCELLED'}
        try: n=build_s_ease_curve_and_bake(context)
        except Exception as e: self.report({'ERROR'}, f"Autocorrect failed: {e}"); return {'CANCELLED'}
        self.report({'INFO'}, f"Autocorrect baked {n} frames. Re-run Validate Motion."); return {'FINISHED'}

class SG_OT_AutocorrectLinear(bpy.types.Operator):
    bl_idname="segway.autocorrect_linear"; bl_label="Autocorrect & Bake (Linear: Rotate-Move-Rotate)"
    def execute(self, context):
        P=context.scene.sg_props
        if P.autocorrect_mode!='LINEAR': self.report({'ERROR'},"Set Autocorrect Mode to 'Linear (Rotate-Move-Rotate)'."); return {'CANCELLED'}
        try: n=build_linear_path_and_bake(context)
        except Exception as e: self.report({'ERROR'}, f"Autocorrect failed: {e}"); return {'CANCELLED'}
        self.report({'INFO'}, f"Linear autocorrect baked {n} frames. Re-run Validate Motion."); return {'FINISHED'}

class SG_OT_RevertAutocorrect(bpy.types.Operator):
    bl_idname="segway.revert_autocorrect"; bl_label="Revert Autocorrect"
    def execute(self, context):
        ch=context.scene.sg_props.chassis
        if not ch: self.report({'ERROR'},"Assign the Chassis."); return {'CANCELLED'}
        ok=_restore_chassis_keys(ch)
        if not ok: self.report({'ERROR'},"No backup found to restore."); return {'CANCELLED'}
        self.report({'INFO'},"Original chassis keyframes restored."); return {'FINISHED'}

def _active_sides(P):
    """('L', 'R') for 2/4/6-wheel configs; ('L', 'R', 'C') for the 3-wheel case."""
    return ('L', 'R', 'C') if _wheel_count(P) == 3 else ('L', 'R')


class SG_OT_AttachDrivers(bpy.types.Operator):
    bl_idname = "segway.attach_drivers"
    bl_label = "Attach Drivers"
    bl_description = "Attach rotation drivers to every configured wheel using the current cache"

    def execute(self, context):
        P = context.scene.sg_props
        if not bpy.app.driver_namespace.get(_driver_key()):
            self.report({'ERROR'}, "Build Cache first (and pass validation)."); return {'CANCELLED'}
        axis_i = _AXIS_INDEX[P.wheel_axis]

        for side in _active_sides(P):
            for obj in _iter_side(P, side):
                # Wipe any existing rotation drivers on this wheel.
                try: obj.driver_remove("rotation_euler", axis_i)
                except Exception: pass
                try:
                    for i in range(4): obj.driver_remove("rotation_quaternion", i)
                except Exception: pass

                if P.rotation_mode == 'EULER':
                    _ensure_xyz_euler(obj)
                    dcurve = obj.driver_add("rotation_euler", axis_i)
                    dcurve.driver.type = 'SCRIPTED'
                    dcurve.driver.expression = f"sg_theta('{side}', frame)"
                else:
                    _ensure_quaternion(obj)
                    # Capture rest BEFORE any driver stubs start evaluating.
                    rq = _obj_rest_quat(obj)
                    obj['rest_w'], obj['rest_x'], obj['rest_y'], obj['rest_z'] = rq
                    for comp in range(4):
                        dcurve = obj.driver_add("rotation_quaternion", comp)
                        dcurve.driver.type = 'SCRIPTED'
                        dcurve.driver.expression = (
                            f"sg_quat_comp_obj('{side}', frame, {comp}, '{P.wheel_axis}', rw, rx, ry, rz)"
                        )
                        for vn, dp in (('rw', '["rest_w"]'), ('rx', '["rest_x"]'),
                                       ('ry', '["rest_y"]'), ('rz', '["rest_z"]')):
                            v = dcurve.driver.variables.new()
                            v.name = vn
                            v.targets[0].id = obj
                            v.targets[0].data_path = dp

        self.report({'INFO'}, "Drivers attached to configured wheels.")
        return {'FINISHED'}


class SG_OT_BuildCache(bpy.types.Operator):
    bl_idname = "segway.build_cache"
    bl_label = "Build Cache"
    bl_description = "Integrate chassis motion into per-frame wheel angles and RPM"

    def execute(self, context):
        try:
            d = build_cache(context)
        except Exception as e:
            self.report({'ERROR'}, str(e)); return {'CANCELLED'}
        if d.get('num_wheels', 2) == 3:
            msg = (f"OK | r={d['radius']:.4f} m | track={d['track']:.4f} m | "
                   f"maxRPM L/R/C {d['max_rpm_L']:.1f}/{d['max_rpm_R']:.1f}/{d['max_rpm_C']:.1f}")
        else:
            msg = (f"OK | r={d['radius']:.4f} m | track={d['track']:.4f} m | "
                   f"maxRPM L/R {d['max_rpm_L']:.1f}/{d['max_rpm_R']:.1f}")
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class SG_OT_Bake(bpy.types.Operator):
    bl_idname = "segway.bake_wheels"
    bl_label = "Bake Wheels"
    bl_description = "Convert the cached wheel angles into per-frame keyframes on every configured wheel"

    def execute(self, context):
        P = context.scene.sg_props
        d = bpy.app.driver_namespace.get(_driver_key())
        if not d:
            self.report({'ERROR'}, "Build Cache first (and pass validation).")
            return {'CANCELLED'}

        f0 = d['f0']; f1 = f0 + len(d['thetaL']) - 1
        axis_i = _AXIS_INDEX[P.wheel_axis]
        ux, uy, uz = _axis_unit(P.wheel_axis)

        for side in _active_sides(P):
            arr = _theta_array_for(d, side)
            for obj in _iter_side(P, side):
                # Drop any existing rotation drivers so bake wins.
                try: obj.driver_remove("rotation_euler", axis_i)
                except Exception: pass
                try:
                    for i in range(4): obj.driver_remove("rotation_quaternion", i)
                except Exception: pass

                if P.rotation_mode == 'EULER':
                    _ensure_xyz_euler(obj)
                else:
                    _ensure_quaternion(obj)

                # Capture per-wheel rest BEFORE we start writing spin values,
                # so every frame uses the same base orientation.
                rest = _obj_rest_quat(obj)
                for f in range(f0, f1 + 1):
                    i = f - f0; th = arr[i]
                    if P.rotation_mode == 'EULER':
                        obj.rotation_euler[axis_i] = th
                        obj.keyframe_insert("rotation_euler", frame=f, index=axis_i)
                    else:
                        half = 0.5 * th; s = sin(half); c = cos(half)
                        q_spin = (c, ux*s, uy*s, uz*s)
                        aw, ax, ay, az = rest
                        bw, bx, by, bz = q_spin
                        q = (aw*bw - ax*bx - ay*by - az*bz,
                             aw*bx + ax*bw + ay*bz - az*by,
                             aw*by - ax*bz + ay*bw + az*bx,
                             aw*bz + ax*by - ay*bx + az*bw)
                        rq = obj.rotation_quaternion
                        rq[0], rq[1], rq[2], rq[3] = q
                        for comp in range(4):
                            obj.keyframe_insert("rotation_quaternion", frame=f, index=comp)

        self.report({'INFO'}, f"Baked frames {f0}..{f1}.")
        return {'FINISHED'}

class SG_OT_Clear(bpy.types.Operator):
    bl_idname = "segway.clear"
    bl_label = "Clear Drivers / Keys"
    bl_description = "Remove rotation drivers and rotation keyframes from every configured wheel"

    def execute(self, context):
        P = context.scene.sg_props
        axis_i = _AXIS_INDEX[P.wheel_axis]
        removed_any = False

        for obj in _iter_all_wheels(P):
            try: obj.driver_remove("rotation_euler", axis_i); removed_any = True
            except Exception: pass
            try:
                for i in range(4): obj.driver_remove("rotation_quaternion", i); removed_any = True
            except Exception: pass

            ad = obj.animation_data
            if ad and ad.action:
                to_del = [fc for fc in _iter_action_fcurves(ad.action)
                          if (fc.data_path == "rotation_euler" and fc.array_index == axis_i)
                          or fc.data_path == "rotation_quaternion"]
                for fc in to_del:
                    _remove_action_fcurve(ad.action, fc); removed_any = True

            if P.rotation_mode == 'EULER':
                try: obj.rotation_euler[axis_i] = 0.0
                except Exception: pass
            else:
                try:
                    rq = obj.rotation_quaternion
                    rq[0], rq[1], rq[2], rq[3] = 1.0, 0.0, 0.0, 0.0
                except Exception: pass

        self.report({'INFO'} if removed_any else {'WARNING'},
                    "Cleared wheel drivers and rotation keyframes."
                    if removed_any else "Nothing to clear on configured wheels.")
        return {'FINISHED'}

# ---------------------- CSV Export ----------------------
class SG_OT_ExportCSV(bpy.types.Operator):
    bl_idname = "segway.export_csv"
    bl_label = "Export CSV"
    bl_description = "Export t, x, y, yaw, thetaR/L (and thetaC on 3-wheel), rateR/L using chosen sampling and units"

    def execute(self, context):
        P = context.scene.sg_props
        d = bpy.app.driver_namespace.get(_driver_key())
        if not d:
            self.report({'ERROR'}, "Build Cache first (and pass validation)."); return {'CANCELLED'}

        include_c = d.get('num_wheels', 2) == 3
        path = bpy.path.abspath(P.csv_path)
        fps = d['fps']; n = len(d['thetaL'])
        ang_k  = 1.0 if P.angle_unit == 'RAD' else 180.0 / pi
        yaw_k  = 180.0 / pi if P.angle_unit == 'DEG' else 1.0
        if   P.angrate_unit == 'RPM': rate_k = 60.0 / (2.0 * pi)
        elif P.angrate_unit == 'RPS': rate_k = 1.0 / (2.0 * pi)
        else:                          rate_k = 180.0 / pi
        len_k = 1.0 if P.length_unit == 'M' else 100.0

        # Convert cached VISUAL thetas (about the wheel's local axis, used by
        # drivers) into PHYSICAL forward-roll thetas by multiplying with each
        # wheel's sign. Result: positive theta / positive rate means the wheel
        # is rolling forward, independently of how the wheel model is oriented.
        sL = float(d.get('signL', 1.0))
        sR = float(d.get('signR', 1.0))
        sC = float(d.get('signC', 1.0))
        thL_phys = [t * sL for t in d['thetaL']]
        thR_phys = [t * sR for t in d['thetaR']]
        thC_phys = [t * sC for t in d.get('thetaC', [0.0] * n)]

        rows = []

        if P.sample_mode == 'FRAME':
            for i in range(n):
                t   = i / fps
                thR = thR_phys[i]; thL = thL_phys[i]; thC = thC_phys[i]
                x   = d['x'][i] * len_k; y = d['y'][i] * len_k
                yaw = d['yaw'][i] * yaw_k
                if i == 0:
                    rateR = rateL = rateC = 0.0
                else:
                    rateR = (thR_phys[i] - thR_phys[i - 1]) * fps * rate_k
                    rateL = (thL_phys[i] - thL_phys[i - 1]) * fps * rate_k
                    rateC = (thC_phys[i] - thC_phys[i - 1]) * fps * rate_k
                rows.append((t, x, y, yaw,
                             thR * ang_k, thL * ang_k, thC * ang_k,
                             rateR, rateL, rateC))
        else:
            try: hz = float(P.fixed_rate)
            except Exception: hz = 100.0
            hz = max(1.0, hz); dt = 1.0 / hz
            total = (n - 1) / fps; k = 0
            while True:
                t = k * dt
                if t > total + 1e-9: break
                idx = t * fps
                thR = _linerp(thR_phys, idx)
                thL = _linerp(thL_phys, idx)
                thC = _linerp(thC_phys, idx)
                x   = _linerp(d['x'], idx) * len_k
                y   = _linerp(d['y'], idx) * len_k
                yaw = _linerp(d['yaw'], idx) * yaw_k
                if k == 0:
                    rateR = rateL = rateC = 0.0
                else:
                    thRprev = rows[-1][4] / ang_k
                    thLprev = rows[-1][5] / ang_k
                    thCprev = rows[-1][6] / ang_k
                    rateR = ((thR - thRprev) / dt) * rate_k
                    rateL = ((thL - thLprev) / dt) * rate_k
                    rateC = ((thC - thCprev) / dt) * rate_k
                rows.append((t, x, y, yaw,
                             thR * ang_k, thL * ang_k, thC * ang_k,
                             rateR, rateL, rateC))
                k += 1

        au = P.angle_unit.lower()
        ru = P.angrate_unit.lower()
        xu = 'x_m' if P.length_unit == 'M' else 'x_cm'
        yu = 'y_m' if P.length_unit == 'M' else 'y_cm'

        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write("# theta/rate are PHYSICAL forward-roll values: "
                        "positive = wheel rolls forward.\n")
                f.write("# Track width is center-to-center of left/right wheels.\n")
                f.write(f"# num_wheels={d.get('num_wheels', 2)}, "
                        f"track_width_m={d.get('track', 0.0):.6f}, "
                        f"swap_lr={P.swap_lr}\n")
                if include_c:
                    f.write(f"t,{xu},{yu},yaw_{au},thetaR_{au},thetaL_{au},thetaC_{au},"
                            f"rateR_{ru},rateL_{ru},rateC_{ru}\n")
                    fmt = "{:.6f},{:.6f},{:.6f},{:.9f},{:.9f},{:.9f},{:.9f},{:.4f},{:.4f},{:.4f}\n"
                    for r in rows:
                        f.write(fmt.format(*r))
                else:
                    f.write(f"t,{xu},{yu},yaw_{au},thetaR_{au},thetaL_{au},rateR_{ru},rateL_{ru}\n")
                    fmt = "{:.6f},{:.6f},{:.6f},{:.9f},{:.9f},{:.9f},{:.4f},{:.4f}\n"
                    for r in rows:
                        # Drop thetaC (index 6) and rateC (index 9).
                        f.write(fmt.format(r[0], r[1], r[2], r[3], r[4], r[5], r[7], r[8]))
        except Exception as e:
            self.report({'ERROR'}, f"Failed to write CSV: {e}"); return {'CANCELLED'}
        self.report({'INFO'}, f"Wrote {path} ({len(rows)} samples).")
        return {'FINISHED'}

# ---------------------- CSV Import ----------------------
class SG_OT_ImportCSV(bpy.types.Operator):
    bl_idname="segway.import_csv"; bl_label="Import Animation from CSV"
    bl_description="Import animation data from CSV file and apply to selected objects"
    
    def execute(self, context):
        P=context.scene.sg_props
        ch=P.chassis
        
        if not ch:
            self.report({'ERROR'}, "Assign the Chassis first."); return {'CANCELLED'}
        
        # Require at least one wheel per side for tire animation to have a target.
        if not _iter_side(P, 'L') or not _iter_side(P, 'R'):
            self.report({'ERROR'}, "Assign at least Left Wheel 01 and Right Wheel 01 before importing.")
            return {'CANCELLED'}
        
        path=bpy.path.abspath(P.csv_import_path)
        if not path or not os.path.exists(path):
            self.report({'ERROR'}, f"CSV file not found: {path}"); return {'CANCELLED'}
        
        try:
            import csv
            
            # Read CSV file
            with open(path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                rows = list(reader)
            
            # Skip header rows
            skip_rows = P.csv_import_skip_rows
            if skip_rows >= len(rows):
                self.report({'ERROR'}, f"Skip rows ({skip_rows}) >= total rows ({len(rows)})"); return {'CANCELLED'}
            
            data_rows = rows[skip_rows:]
            if not data_rows:
                self.report({'ERROR'}, "No data rows found after skipping header rows"); return {'CANCELLED'}
            
            # Parse header row (first data row)
            header = data_rows[0]
            data_rows = data_rows[1:]  # Remove header from data
            
            # Find column indices - match your actual CSV format
            col_indices = {}
            for i, col_name in enumerate(header):
                col_name = col_name.strip().lower()
                
                # Match your actual CSV column names
                if col_name == 't':
                    col_indices['frame'] = i  # Use 't' as frame number
                elif col_name == 'x_m':
                    col_indices['x'] = i
                elif col_name == 'y_m':
                    col_indices['y'] = i
                elif col_name == 'yaw_rad':
                    col_indices['yaw'] = i  # This is rotation, not z position
                elif col_name == 'thetar_rad':
                    col_indices['thetaR'] = i
                elif col_name == 'thetal_rad':
                    col_indices['thetaL'] = i
                # Also support the original format for compatibility
                elif col_name == 'frame':
                    col_indices['frame'] = i
                elif col_name == 'x':
                    col_indices['x'] = i
                elif col_name == 'y':
                    col_indices['y'] = i
                elif col_name == 'z':
                    col_indices['z'] = i
                elif col_name == 'euler_x':
                    col_indices['euler_x'] = i
                elif col_name == 'euler_y':
                    col_indices['euler_y'] = i
                elif col_name == 'euler_z':
                    col_indices['euler_z'] = i
                elif col_name == 'quat_w':
                    col_indices['quat_w'] = i
                elif col_name == 'quat_x':
                    col_indices['quat_x'] = i
                elif col_name == 'quat_y':
                    col_indices['quat_y'] = i
                elif col_name == 'quat_z':
                    col_indices['quat_z'] = i
                elif col_name == 'thetar':
                    col_indices['thetaR'] = i
                elif col_name == 'thetal':
                    col_indices['thetaL'] = i
                elif col_name in ('thetac', 'thetac_rad'):
                    col_indices['thetaC'] = i
            
            # Check required columns - your CSV format has t, x_m, y_m, yaw_rad
            required_cols = ['frame', 'x', 'y']  # z is optional, yaw_rad is rotation
            missing_cols = [col for col in required_cols if col not in col_indices]
            if missing_cols:
                self.report({'ERROR'}, f"Missing required columns: {missing_cols}")
                self.report({'ERROR'}, f"Available columns: {[header[i] for i in range(len(header))]}")
                self.report({'ERROR'}, f"Try adjusting 'Skip Rows from Top' setting (currently {skip_rows})")
                return {'CANCELLED'}
            
            # Ensure action + slot, then clear existing chassis animation
            act = _ensure_action_for(ch, action_name="ImportedAnimation")
            for fc in list(_iter_action_fcurves(act)):
                if fc.data_path in ("location", "rotation_euler", "rotation_quaternion"):
                    _remove_action_fcurve(act, fc)

            # Use the scene's FPS and start frame so time columns align with
            # the current scene, not with an arbitrary 30 FPS / frame-1 origin.
            scn = context.scene
            scene_fps = scn.render.fps / max(1e-9, scn.render.fps_base)
            base_frame = int(scn.frame_start)

            # Process data rows
            frames_processed = 0
            for row in data_rows:
                if len(row) <= max(col_indices.values()):
                    continue  # Skip incomplete rows

                try:
                    # Parse frame number (t column in CSV contains time in seconds)
                    time_seconds = float(row[col_indices['frame']])
                    frame = int(round(time_seconds * scene_fps)) + base_frame
                    
                    # Parse position
                    x = float(row[col_indices['x']])
                    y = float(row[col_indices['y']])
                    
                    # Handle z position - use 0 if not available
                    if 'z' in col_indices:
                        z = float(row[col_indices['z']])
                    else:
                        z = 0.0  # Default z position
                    
                    # Set chassis location
                    ch.location = (x, y, z)
                    ch.keyframe_insert("location", frame=frame, index=-1)
                    
                    # Handle rotation - your CSV has yaw_rad
                    if 'yaw' in col_indices:
                        # Use yaw_rad for rotation
                        _ensure_xyz_euler(ch)
                        yaw = float(row[col_indices['yaw']])
                        ch.rotation_euler = (0.0, 0.0, yaw)  # Only Z rotation (yaw)
                        ch.keyframe_insert("rotation_euler", frame=frame, index=-1)
                    elif all(col in col_indices for col in ['quat_w', 'quat_x', 'quat_y', 'quat_z']):
                        # Use quaternion rotation
                        _ensure_quaternion(ch)
                        qw = float(row[col_indices['quat_w']])
                        qx = float(row[col_indices['quat_x']])
                        qy = float(row[col_indices['quat_y']])
                        qz = float(row[col_indices['quat_z']])
                        ch.rotation_quaternion = (qw, qx, qy, qz)
                        ch.keyframe_insert("rotation_quaternion", frame=frame, index=-1)
                    elif all(col in col_indices for col in ['euler_x', 'euler_y', 'euler_z']):
                        # Use euler rotation
                        _ensure_xyz_euler(ch)
                        ex = float(row[col_indices['euler_x']])
                        ey = float(row[col_indices['euler_y']])
                        ez = float(row[col_indices['euler_z']])
                        ch.rotation_euler = (ex, ey, ez)
                        ch.keyframe_insert("rotation_euler", frame=frame, index=-1)
                    
                    frames_processed += 1
                    
                except (ValueError, IndexError) as e:
                    continue  # Skip invalid rows
            
            # Apply tire rotation to wheel objects if thetaR / thetaL columns exist.
            # thetaC is applied to the caster if the 3-wheel config is active
            # and a thetaC column is present.
            if 'thetaR' in col_indices and 'thetaL' in col_indices:
                axis_i = _AXIS_INDEX[P.wheel_axis]
                ux, uy, uz = _axis_unit(P.wheel_axis)

                sides_present = ['L', 'R']
                if _wheel_count(P) == 3 and 'thetaC' in col_indices and _iter_side(P, 'C'):
                    sides_present.append('C')

                # Clear existing wheel animation and capture each wheel's rest
                # quaternion ONCE so subsequent spin composition doesn't drift.
                wheel_rest = {}  # id(obj) -> (w,x,y,z)
                for side in sides_present:
                    for obj in _iter_side(P, side):
                        if P.rotation_mode == 'EULER':
                            _ensure_xyz_euler(obj)
                            try: obj.driver_remove("rotation_euler", axis_i)
                            except Exception: pass
                        else:
                            _ensure_quaternion(obj)
                            try:
                                for i in range(4): obj.driver_remove("rotation_quaternion", i)
                            except Exception: pass

                        ad = obj.animation_data
                        if ad and ad.action:
                            to_del = [fc for fc in _iter_action_fcurves(ad.action)
                                    if (fc.data_path == "rotation_euler" and fc.array_index == axis_i)
                                    or fc.data_path == "rotation_quaternion"]
                            for fc in to_del:
                                _remove_action_fcurve(ad.action, fc)

                        wheel_rest[id(obj)] = _obj_rest_quat(obj)

                # Apply tire rotations
                for row in data_rows:
                    if len(row) <= max(col_indices.values()):
                        continue

                    try:
                        time_seconds = float(row[col_indices['frame']])
                        frame = int(round(time_seconds * scene_fps)) + base_frame
                        thetas = {
                            'L': float(row[col_indices['thetaL']]),
                            'R': float(row[col_indices['thetaR']]),
                        }
                        if 'C' in sides_present:
                            thetas['C'] = float(row[col_indices['thetaC']])

                        for side in sides_present:
                            theta = thetas[side]
                            half = 0.5 * theta
                            s = sin(half); c = cos(half)
                            q_spin = (c, ux*s, uy*s, uz*s)
                            for obj in _iter_side(P, side):
                                if P.rotation_mode == 'EULER':
                                    obj.rotation_euler[axis_i] = theta
                                    obj.keyframe_insert("rotation_euler", frame=frame, index=axis_i)
                                else:
                                    aw, ax, ay, az = wheel_rest.get(id(obj), (1.0, 0.0, 0.0, 0.0))
                                    bw, bx, by, bz = q_spin
                                    q = (aw*bw - ax*bx - ay*by - az*bz,
                                         aw*bx + ax*bw + ay*bz - az*by,
                                         aw*by - ax*bz + ay*bw + az*bx,
                                         aw*bz + ax*by - ay*bx + az*bw)
                                    rq = obj.rotation_quaternion
                                    rq[0], rq[1], rq[2], rq[3] = q
                                    for comp in range(4):
                                        obj.keyframe_insert("rotation_quaternion", frame=frame, index=comp)

                    except (ValueError, IndexError):
                        continue
            
            self.report({'INFO'}, f"Imported {frames_processed} frames from CSV. Chassis and wheel animations applied.")
            return {'FINISHED'}
            
        except Exception as e:
            self.report({'ERROR'}, f"Failed to import CSV: {e}")
            return {'CANCELLED'}

# ---------------------- Keyframe Export ----------------------
def _collect_keyframes(obj,fmin,fmax):
    frames=set(); ad=obj.animation_data
    if ad and ad.action:
        for fc in _iter_action_fcurves(ad.action):
            if fc.data_path in ("location","rotation_euler","rotation_quaternion"):
                for kp in fc.keyframe_points:
                    fr=int(round(kp.co[0]))
                    if fmin<=fr<=fmax: frames.add(fr)
    return sorted(frames)

class SG_OT_ExportKeyframes(bpy.types.Operator):
    bl_idname="segway.export_keyframes"; bl_label="Export Keyframes"
    def execute(self, context):
        scn=context.scene; P=scn.sg_props; ch=P.chassis
        if not ch: self.report({'ERROR'},"Assign the Chassis."); return {'CANCELLED'}
        d=bpy.app.driver_namespace.get(_driver_key())
        if not d: self.report({'ERROR'},"Build Cache first (and pass validation)."); return {'CANCELLED'}
        f0=d['f0']; f1=f0+len(d['thetaL'])-1; fps=d['fps']
        kf=_collect_keyframes(ch, scn.frame_start, scn.frame_end) or [scn.frame_start, scn.frame_end]
        kf=[f for f in kf if f0<=f<=f1]
        if not kf: self.report({'ERROR'},"No keyed frames fall inside the cache/frame range."); return {'CANCELLED'}
        ang_k=1.0 if P.other_angle_unit=='RAD' else (180.0/pi)
        path=bpy.path.abspath(P.other_export_path)
        rows=[]; deps=context.evaluated_depsgraph_get(); prev=scn.frame_current
        try:
            for f in kf:
                scn.frame_set(f); deps.update()
                mw=ch.matrix_world; loc=mw.translation; eul=mw.to_euler('XYZ'); quat=mw.to_quaternion()
                idx=f-f0; thR=d['thetaR'][idx]*ang_k; thL=d['thetaL'][idx]*ang_k
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
            if P.other_export_format=='CSV':
                with open(path,"w",encoding="utf-8") as f:
                    f.write(f"# keyframes_only=1, angle_unit={P.other_angle_unit}\n")
                    f.write("frame,fps,time_s,x,y,z,euler_x,euler_y,euler_z,quat_w,quat_x,quat_y,quat_z,thetaR,thetaL\n")
                    for r in rows:
                        f.write("{frame},{fps:.6f},{time_s:.6f},{x:.6f},{y:.6f},{z:.6f},{euler_x:.9f},{euler_y:.9f},{euler_z:.9f},{quat_w:.9f},{quat_x:.9f},{quat_y:.9f},{quat_z:.9f},{thetaR:.9f},{thetaL:.9f}\n".format(**r))
            else:
                out={"meta":{"angle_unit":P.other_angle_unit,"fps":fps,"swap_lr":P.swap_lr,
                             "frame_start":int(scn.frame_start),"frame_end":int(scn.frame_end)},
                     "samples":rows}
                with open(path,"w",encoding="utf-8") as f: json.dump(out,f,indent=2)
        except Exception as e:
            self.report({'ERROR'}, f"Failed to write: {e}"); return {'CANCELLED'}
        self.report({'INFO'}, f"Keyframes exported to {path} ({len(rows)} rows)"); return {'FINISHED'}

# ---------------------- UI ----------------------
class SG_PT_Panel(bpy.types.Panel):
    bl_label = "True RoboAnimator"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "True RoboAnimator"

    def _section(self, layout, P, flag, title, icon='NONE'):
        """Collapsible section. Returns the inner column when open, else None."""
        box = layout.box()
        row = box.row(align=True)
        is_open = getattr(P, flag)
        row.prop(P, flag, text="",
                 icon='TRIA_DOWN' if is_open else 'TRIA_RIGHT',
                 emboss=False)
        row.label(text=title, icon=icon)
        return box.column() if is_open else None

    def draw(self, context):
        layout = self.layout
        P = context.scene.sg_props
        layout.use_property_split = True
        layout.use_property_decorate = False

        # ---- How to Use ----
        col = self._section(layout, P, "show_instructions", "How to Use", icon='HELP')
        if col:
            c = col.column(align=True)
            c.label(text="1. Assign chassis and wheels.")
            c.label(text="2. Keyframe chassis motion.")
            c.label(text="3. Validate, then Autocorrect.")
            c.label(text="4. Build Cache, then Attach Drivers or Bake.")
            c.label(text="5. Export.")

        # ---- Viewport Helpers (near the top so it's easy to find) ----
        col = self._section(layout, P, "show_viewport", "Viewport Helpers", icon='OVERLAY')
        if col:
            col.prop(P, "vis_enabled")
            sub = col.column(align=True)
            sub.enabled = P.vis_enabled
            sub.prop(P, "vis_show_forward")
            sub.prop(P, "vis_show_wheels")
            sub.prop(P, "vis_show_raw_path")
            sub.prop(P, "vis_show_solution_path")

        # ---- Rig Setup ----
        col = self._section(layout, P, "show_setup", "Rig Setup", icon='OUTLINER_OB_ARMATURE')
        if col:
            col.prop(P, "chassis")
            col.separator()
            col.prop(P, "num_wheels", expand=True)
            col.separator()
            n = _wheel_count(P)
            g = col.column(align=True)
            g.prop(P, "wheel_l_01")
            if n >= 4: g.prop(P, "wheel_l_02")
            if n == 6: g.prop(P, "wheel_l_03")
            g.prop(P, "wheel_r_01")
            if n >= 4: g.prop(P, "wheel_r_02")
            if n == 6: g.prop(P, "wheel_r_03")
            if n == 3: g.prop(P, "wheel_caster")
            col.separator()
            col.prop(P, "swap_lr")

        # ---- Calibration ----
        col = self._section(layout, P, "show_calibration", "Calibration", icon='DRIVER_DISTANCE')
        if col:
            col.prop(P, "auto_track_width")
            r = col.row(); r.enabled = not P.auto_track_width
            r.prop(P, "track_width")
            if P.auto_track_width:
                col.label(text=f"Auto: {_effective_track_width(P):.4f} m", icon='INFO')
            col.separator()
            col.prop(P, "auto_radius")
            if not P.auto_radius:
                col.prop(P, "wheel_radius")
            col.separator()
            col.prop(P, "wheel_axis")
            col.prop(P, "rotation_mode")

        # ---- Motion Path ----
        col = self._section(layout, P, "show_motion", "Motion Path", icon='CURVE_BEZCURVE')
        if col:
            col.prop(P, "body_forward_axis")
            col.prop(P, "side_tol")
            col.separator()
            col.prop(P, "autocorrect_mode")
            r = col.row(); r.enabled = (P.autocorrect_mode == 'SEASE')
            r.prop(P, "bezier_tangent_start")
            r = col.row(); r.enabled = (P.autocorrect_mode == 'SEASE')
            r.prop(P, "bezier_tangent_end")
            r = col.row(); r.enabled = (P.autocorrect_mode == 'LINEAR')
            r.prop(P, "linear_rotation_fraction")
            col.separator()
            col.prop(P, "speed_profile")
            r = col.row(); r.enabled = (P.speed_profile == 'CONSTANT')
            r.prop(P, "constant_ramp_frames")
            r = col.row(); r.enabled = (P.speed_profile == 'GLOBAL_EASE')
            r.prop(P, "timeline_ease_frames")
            r = col.row(); r.enabled = (P.speed_profile == 'PER_KEY_EASE')
            r.prop(P, "segment_ease_frames")
            col.separator()
            r = col.row(align=True)
            r.operator("segway.validate_motion",  icon='CHECKMARK')
            r.operator("segway.autocorrect_bake", icon='MOD_CURVE')
            col.operator("segway.revert_autocorrect", icon='LOOP_BACK')

        # ---- Wheel Drivers ----
        col = self._section(layout, P, "show_drivers", "Wheel Drivers", icon='DRIVER')
        if col:
            col.prop(P, "max_rpm")
            col.prop(P, "max_ang_accel_rpm_s")
            col.separator()
            r = col.row(align=True)
            r.operator("segway.build_cache",    icon='FILE_REFRESH')
            r.operator("segway.attach_drivers", icon='CONSTRAINT')
            r = col.row(align=True)
            r.operator("segway.bake_wheels", icon='REC')
            r.operator("segway.clear",       icon='TRASH')

        # ---- Import / Export (combined) ----
        col = self._section(layout, P, "show_io", "Import / Export", icon='FILE_TICK')
        if col:
            col.prop(P, "io_mode", expand=True)
            col.separator()
            if P.io_mode == 'IMPORT':
                col.prop(P, "csv_import_path",      text="File")
                col.prop(P, "csv_import_skip_rows", text="Skip Rows")
                col.operator("segway.import_csv", icon='IMPORT')
            elif P.io_mode == 'KEYS':
                col.prop(P, "other_export_path",   text="File")
                col.prop(P, "other_export_format", text="Format")
                col.prop(P, "other_angle_unit",    text="Angle")
                col.operator("segway.export_keyframes", icon='EXPORT')
            else:  # 'CSV'
                col.prop(P, "csv_path",     text="File")
                col.prop(P, "sample_mode",  text="Sampling")
                if P.sample_mode == 'FIXED':
                    col.prop(P, "fixed_rate", text="Rate")
                col.prop(P, "angle_unit",   text="Angle")
                col.prop(P, "angrate_unit", text="Angular Rate")
                col.prop(P, "length_unit",  text="Length")
                col.operator("segway.export_csv", icon='EXPORT')

# ---------------------- register / unregister ----------------------
classes=(
    SG_Props,
    SG_OT_ValidateMotion,
    SG_OT_AutocorrectBake,
    SG_OT_AutocorrectSEase,
    SG_OT_AutocorrectLinear,
    SG_OT_RevertAutocorrect,
    SG_OT_AttachDrivers,
    SG_OT_BuildCache,
    SG_OT_Bake,
    SG_OT_Clear,
    SG_OT_ImportCSV,
    SG_OT_ExportCSV,
    SG_OT_ExportKeyframes,
    SG_PT_Panel,
)

def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.sg_props = bpy.props.PointerProperty(type=SG_Props)
    bpy.app.driver_namespace['sg_theta'] = sg_theta
    bpy.app.driver_namespace['sg_quat_comp'] = sg_quat_comp
    bpy.app.driver_namespace['sg_quat_comp_obj'] = sg_quat_comp_obj
    _register_draw_handler()
    _tag_viewport_redraw()

def unregister():
    _unregister_draw_handler()
    for c in reversed(classes):
        bpy.utils.unregister_class(c)
    for k in ('sg_theta', 'sg_quat_comp', 'sg_quat_comp_obj', _driver_key()):
        bpy.app.driver_namespace.pop(k, None)
    del bpy.types.Scene.sg_props

if __name__=="__main__":
    register()
