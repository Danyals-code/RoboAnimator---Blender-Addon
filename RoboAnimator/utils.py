# utils.py — shared helpers (no class registration)

import bpy
import json
from math import pi, sin, cos, sqrt, floor, atan2
from mathutils import Vector

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

# ---------------------- Bezier / curvature tools ----------------------
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

# ---------------------- True trapezoid mapping ----------------------
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
        return 0.5 * v_peak * (tau*tau) / f

    if tau < 1.0 - f:
        s_f = 0.5 * v_peak * f
        return s_f + v_peak * (tau - f)

    u = (tau - (1.0 - f)) / f
    s_start = 0.5 * v_peak * f + v_peak * (1.0 - 2.0*f)
    return s_start + v_peak * f * (u - 0.5*u*u)

# ---------------------- Bezier helpers for XY path ----------------------
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

def _lerp(a,b,t):
    return a*(1.0-t)+b*t


# ---------------------- Chassis Key Poses ----------------------

def _get_chassis_key_poses(P, ch):
    frames = set()
    ad = ch.animation_data
    if ad and ad.action:
        for fc in ad.action.fcurves:
            if fc.data_path in ("location", "rotation_euler"):
                for kp in fc.keyframe_points:
                    frames.add(int(round(kp.co[0])))

    if not frames:
        return []

    scn = bpy.context.scene
    deps = bpy.context.evaluated_depsgraph_get()
    out = []
    for f in sorted(frames):
        scn.frame_set(f)
        deps.update()
        mw = ch.matrix_world
        loc = mw.translation
        yaw = ch.matrix_world.to_euler('XYZ').z
        heading = yaw_to_heading(P, yaw)
        out.append((f, loc.x, loc.y, loc.z, yaw, heading))
    return out


# ---------------------- backup / restore chassis keys ----------------------
def _collect_fcurves(obj):
    ad=obj.animation_data
    if not (ad and ad.action): return []
    return [fc for fc in ad.action.fcurves if fc.data_path in ("location","rotation_euler")]

def _backup_chassis_keys(ch):
    data={}
    for fc in _collect_fcurves(ch):
        key=f"{fc.data_path}[{fc.array_index}]"
        data[key]=[[float(kp.co[0]), float(kp.co[1])] for kp in fc.keyframe_points]
    key=f"{_BACKUP_KEY}_{bpy.context.scene.name}_{ch.name}"
    txt=bpy.data.texts.get(key) or bpy.data.texts.new(key)
    txt.clear(); txt.write(json.dumps(data))
    return True

def _restore_chassis_keys(ch):
    key=f"{_BACKUP_KEY}_{bpy.context.scene.name}_{ch.name}"
    txt=bpy.data.texts.get(key)
    if not txt or len(txt.as_string())==0: return False
    try: data=json.loads(txt.as_string())
    except Exception: return False
    if not ch.animation_data: ch.animation_data_create()
    if not ch.animation_data.action:
        ch.animation_data.action=bpy.data.actions.new(name="ChassisAction")
    ad=ch.animation_data
    if ad and ad.action:
        for fc in list(ad.action.fcurves):
            if fc.data_path in ("location","rotation_euler"):
                try: ad.action.fcurves.remove(fc)
                except Exception: pass
    _ensure_xyz_euler(ch)
    for key,rows in data.items():
        if key.startswith("location["):
            path="location"; idx=int(key.split("[")[1][0])
        elif key.startswith("rotation_euler["):
            path="rotation_euler"; idx=int(key.split("[")[1][0])
        else: continue
        fc=ch.animation_data.action.fcurves.new(data_path=path,index=idx)
        for fr,val in rows:
            fc.keyframe_points.insert(frame=fr, value=val, options={'FAST'})
    txt.clear(); return True

# ---------------------- side helpers ----------------------
def _col_for_side(P, side):
    return (P.left_collection if side=='L' else P.right_collection) if not P.swap_lr else \
           (P.right_collection if side=='L' else P.left_collection)

def _iter_side(P, side):
    col=_col_for_side(P,side)
    if not col: return []
    return [o for o in col.objects if o.type in {'MESH','EMPTY'}]

# ---------------------- body basis from heading + forward-axis ----------------------
def _body_basis_from_yaw(theta, forward_axis):
    c=cos(theta); s=sin(theta)
    if   forward_axis=='+Y': fwd=(-s, c)
    elif forward_axis=='-Y': fwd=( s,-c)
    elif forward_axis=='-X': fwd=(-c,-s)
    else:                    fwd=( c, s)  # +X
    lat=(-fwd[1], fwd[0])
    return fwd, lat

# ---------------------- Edge ease tools ----------------------
def _edge_ease_progress(tau, ease_frac):
    """Symmetric edge ease on [0,1], continuous at the joins (C1)."""
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

# ---------------------- rest quaternion helper ----------------------
def _obj_rest_quat(obj):
    if not obj: return (1.0,0.0,0.0,0.0)
    if getattr(obj,'rotation_mode','XYZ')=='QUATERNION':
        q=(obj.rotation_quaternion[0], obj.rotation_quaternion[1], obj.rotation_quaternion[2], obj.rotation_quaternion[3])
    else:
        qq=obj.rotation_euler.to_quaternion(); q=(qq.w,qq.x,qq.y,qq.z)
    n=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]) or 1.0
    return (q[0]/n, q[1]/n, q[2]/n, q[3]/n)

# ---------------------- wheel selector ----------------------
def _iter_wheels(P, side):
    """Return a list of wheel objects for 'L' or 'R' using explicit pointers first, legacy collections second."""
    wheels = []
    sys = getattr(P, "wheel_system", "2W")

    if sys == "4W":
        names = (["wheel_fl","wheel_rl"] if side == 'L' else ["wheel_fr","wheel_rr"])
        for n in names:
            o = getattr(P, n, None)
            if isinstance(o, bpy.types.Object):
                wheels.append(o)
    else:  # 2W
        n = "wheel_left" if side == 'L' else "wheel_right"
        o = getattr(P, n, None)
        if isinstance(o, bpy.types.Object):
            wheels.append(o)

    # Legacy fallback to collections if explicit not set
    if not wheels:
        wheels = _iter_side(P, side)

    # Dedup while preserving order
    out, seen = [], set()
    for o in wheels:
        if o and o.name not in seen:
            seen.add(o.name); out.append(o)
    return out


# ---------------------- public symbols ----------------------
__all__ = [
    "_AXIS_INDEX","_DEFKEY","_BACKUP_KEY","_driver_key",
    "_wrap","_unwrap","_axis_unit","_auto_radius",
    "_ensure_xyz_euler","_ensure_quaternion","_linerp",
    "yaw_to_heading","heading_to_yaw",
    "_bezier_eval","_ease_in_out_cubic","_trapezoid_s",
    "_bezier_point_xy","_bezier_tangent_xy","_build_arc_lut_norm_total_xy",
    "_arc_to_t_from_lut","_lerp",
    "_collect_fcurves","_backup_chassis_keys","_restore_chassis_keys",
    "_col_for_side","_iter_side","_body_basis_from_yaw",
    "_edge_ease_progress","_edge_ease_progress_asym",
    "_obj_rest_quat",
    "_iter_wheels",
    "_get_chassis_key_poses",
]
