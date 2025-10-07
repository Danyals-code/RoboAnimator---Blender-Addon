bl_info = {
    "name": "True RoboAnimator",
    "author": "Danyal S.",
    "version": (1, 0, 0),
    "blender": (4, 2, 0),
    "location": "3D Viewport > N-Panel > True RoboAnimator",
    "description": "Engineering-accurate animation toolkit for differential-drive robots.",
    "category": "Animation",
}


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

# ---------------------- validation (no sideways slip per frame) ----------------------
def analyze_motion(context):
    P=context.scene.sg_props; ch=P.chassis
    if not ch: raise RuntimeError("Assign the Chassis.")
    if P.track_width<=0: raise RuntimeError("Track width must be > 0.")
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
    return {'f0':f0,'fps':fps,'track':P.track_width,'violations':violations,'violation_frames':v_frames,'side_tol':P.side_tol,'dt':dt}

# ---------------------- gather keyed poses ----------------------
def _get_chassis_key_poses(P,ch):
    frames=set()
    ad=ch.animation_data
    if ad and ad.action:
        for fc in ad.action.fcurves:
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
def _bake_chassis_frames_from_heading_samples(P,ch,samples):
    _ensure_xyz_euler(ch)
    scn=bpy.context.scene; deps=bpy.context.evaluated_depsgraph_get()
    ad=ch.animation_data
    if ad and ad.action:
        for fc in list(ad.action.fcurves):
            if fc.data_path in ("location","rotation_euler"):
                try: ad.action.fcurves.remove(fc)
                except Exception: pass
    for f,x,y,z,h in samples:
        ch.location.x=x; ch.location.y=y; ch.location.z=z
        ch.rotation_euler[0]=0.0; ch.rotation_euler[1]=0.0
        ch.rotation_euler[2]=heading_to_yaw(P,h)
        ch.keyframe_insert("location", frame=f, index=-1)
        ch.keyframe_insert("rotation_euler", frame=f, index=-1)
    deps.update(); scn.frame_set(scn.frame_start); deps.update()

# ---------------------- S-curve build + bake ----------------------
def _build_s_ease_curve_segment(poseA, poseB, rho, tangent_scale):
    (x0,y0,h0) = poseA
    (x3,y3,h3) = poseB

    chord = atan2(y3 - y0, x3 - x0)
    def _align(h, ref):
        a = _wrap(h - ref)
        return h if abs(a) <= pi/2 else _wrap(h + pi)
    h0 = _align(h0, chord)
    h3 = _align(h3, chord)

    P0=(x0,y0); P3=(x3,y3)
    dx=x3-x0; dy=y3-y0; dist=max(1e-6, sqrt(dx*dx+dy*dy))

    L=min(dist*tangent_scale, 5.0*rho, 0.49*dist)
    if L<1e-8: L=1e-8

    P1=(x0 + L*cos(h0), y0 + L*sin(h0))
    P2=(x3 - L*cos(h3), y3 - L*sin(h3))

    max_k_allow=(1.0/rho)*1.02
    for _ in range(12):
        exceeds=False
        for t in (0.15,0.35,0.5,0.65,0.85):
            _,_,_,k=_bezier_eval(P0,P1,P2,P3,t)
            if k>max_k_allow: exceeds=True; break
        if not exceeds: break
        L*=0.8; L=max(min(L,0.49*dist),1e-8)
        P1=(x0 + L*cos(h0), y0 + L*sin(h0))
        P2=(x3 - L*cos(h3), y3 - L*sin(h3))
    return P0,P1,P2,P3

def build_s_ease_curve_and_bake(context):
    P=context.scene.sg_props; ch=P.chassis
    if not ch: raise RuntimeError("Assign the Chassis.")
    scn=context.scene
    if not (ch.animation_data and ch.animation_data.action):
        raise RuntimeError("Chassis has no animation to autocorrect.")

    rho=max(P.track_width*0.5, 1e-5)
    tangent_scale=P.bezier_tangent_scale

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
        P0,P1,P2,P3=_build_s_ease_curve_segment((xA,yA,hA),(xB,yB,hB), rho, tangent_scale)
        lut_norm,L=_build_arc_lut_norm_total_xy(P0,P1,P2,P3,steps=128)
        if L<=1e-12: continue
        segs.append({"P0":P0,"P1":P1,"P2":P2,"P3":P3,"zA":zA,"zB":zB,"L":L,"lut":lut_norm})
        total_len+=L

    import bisect
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

    import bisect
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

# ---------------------- cache build for wheel drivers ----------------------
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

    colL=_col_for_side(P,'L'); colR=_col_for_side(P,'R')
    wL0=colL.objects[0] if (colL and len(colL.objects)>0) else None
    wR0=colR.objects[0] if (colR and len(colR.objects)>0) else None
    restL=_obj_rest_quat(wL0); restR=_obj_rest_quat(wR0)

    signL=+1.0 if P.sign_l=='PLUS' else -1.0
    signR=+1.0 if P.sign_r=='PLUS' else -1.0
    try:
        if P.wheel_forward_invert:
            signL*=-1.0; signR*=-1.0
    except Exception:
        pass
    b=P.track_width
    if P.auto_radius:
        w_ref=wL0 or wR0
        if not w_ref: raise RuntimeError("Put at least one wheel object into the Right/Left collections.")
        radius=_auto_radius(w_ref, P.wheel_axis)
    else:
        radius=P.wheel_radius
    if radius<=0: raise RuntimeError("Wheel radius must be > 0.")

    scn=context.scene; deps=context.evaluated_depsgraph_get()
    fps=scn.render.fps/scn.render.fps_base
    dt=1.0/float(fps); f0=scn.frame_start; f1=scn.frame_end
    thetaL=[0.0]; thetaR=[0.0]; pos_x=[]; pos_y=[]; yaw_z=[]

    scn.frame_set(f0); deps.update()
    prev_loc=ch.matrix_world.translation.copy()
    prev_yaw=ch.matrix_world.to_euler('XYZ').z
    pos_x.append(prev_loc.x); pos_y.append(prev_loc.y); yaw_z.append(prev_yaw)
    tL=tR=0.0
    rpmL_list=[0.0]; rpmR_list=[0.0]

    for f in range(f0+1, f1+1):
        scn.frame_set(f); deps.update()
        loc=ch.matrix_world.translation
        yaw=_unwrap(prev_yaw, ch.matrix_world.to_euler('XYZ').z)
        yaw_mid=_wrap((prev_yaw+yaw)*0.5)

        dp_x=loc.x-prev_loc.x; dp_y=loc.y-prev_loc.y
        fwd,_lat=_body_basis_from_yaw(yaw_mid, P.body_forward_axis)
        dx=dp_x*fwd[0] + dp_y*fwd[1]

        if P.wheel_forward_invert:
            dx = -dx

        dpsi=yaw - prev_yaw
        dsL=dx - 0.5*b*dpsi
        dsR=dx + 0.5*b*dpsi
        dthL=(dsL/radius)*signL; dthR=(dsR/radius)*signR
        tL+=dthL; tR+=dthR
        thetaL.append(tL); thetaR.append(tR)
        pos_x.append(loc.x); pos_y.append(loc.y); yaw_z.append(yaw)

        rpmL=(dthL*fps)*60.0/(2.0*pi); rpmR=(dthR*fps)*60.0/(2.0*pi)
        rpmL_list.append(rpmL); rpmR_list.append(rpmR)

        prev_loc=loc.copy(); prev_yaw=yaw

    # limits
    max_rpm_limit   = float(getattr(P, "max_rpm", 0.0))
    max_arpm_s_limit= float(getattr(P, "max_ang_accel_rpm_s", 0.0))

    if max_rpm_limit > 0.0:
        over=[]
        for i,(rL,rR) in enumerate(zip(rpmL_list, rpmR_list)):
            if abs(rL)>max_rpm_limit or abs(rR)>max_rpm_limit:
                over.append(f0+i)
        if over:
            head=", ".join(map(str, over[:12])) + (" …" if len(over)>12 else "")
            raise RuntimeError(f"Wheel RPM limit exceeded (>{max_rpm_limit:.1f} rpm) at frames: {head}")

    if max_arpm_s_limit > 0.0:
        over=[]
        for i in range(1,len(rpmL_list)):
            aL=(rpmL_list[i]-rpmL_list[i-1]) * fps  # rpm per second
            aR=(rpmR_list[i]-rpmR_list[i-1]) * fps
            if abs(aL)>max_arpm_s_limit or abs(aR)>max_arpm_s_limit:
                over.append(f0+i)
        if over:
            head=", ".join(map(str, over[:12])) + (" …" if len(over)>12 else "")
            raise RuntimeError(f"Wheel angular-accel limit exceeded (>{max_arpm_s_limit:.1f} rpm/s) at frames: {head}")

    data={'f0':f0,'fps':fps,'thetaL':thetaL,'thetaR':thetaR,'x':pos_x,'y':pos_y,'yaw':yaw_z,
          'restL':restL,'restR':restR,'radius':radius,'track':b,
          'max_rpm_L':max(abs(r) for r in rpmL_list),'max_rpm_R':max(abs(r) for r in rpmR_list),
          'violations':0,'violation_frames':[]}
    bpy.app.driver_namespace[_driver_key()] = data
    return data

# ---------------------- driver functions (for expressions) ----------------------
def sg_theta(side, frame):
    d=bpy.app.driver_namespace.get(_driver_key())
    if not d: return 0.0
    i=int(frame - d['f0']); i=max(0, min(i, len(d['thetaL'])-1))
    return d['thetaL'][i] if side=='L' else d['thetaR'][i]

def sg_quat_comp(side, frame, comp_index, axis_char):
    d=bpy.app.driver_namespace.get(_driver_key())
    if not d: return (1.0,0.0,0.0,0.0)[int(comp_index)%4]
    th=sg_theta(side, frame)
    ux,uy,uz=_axis_unit(axis_char); h=0.5*th; s=sin(h); c=cos(h)
    q_spin=(c,ux*s,uy*s,uz*s)
    q_rest=d['restL'] if side=='L' else d['restR']
    aw,ax,ay,az=q_rest; bw,bx,by,bz=q_spin
    q=(aw*bw-ax*bx-ay*by-az*bz,
       aw*bx+ax*bw+ay*bz-az*by,
       aw*by-ax*bz+ay*bw+az*bx,
       aw*bz+ax*by-ay*bx+az*bw)
    return q[int(comp_index)%4]

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

# ---------------------- properties ----------------------
class SG_Props(bpy.types.PropertyGroup):
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
    wheel_axis: bpy.props.EnumProperty(name="Wheel Rotation Axis", items=[('X','X',''),('Y','Y',''),('Z','Z','')], default='X')
    rotation_mode: bpy.props.EnumProperty(name="Rotation Mode", items=[('EULER','Euler',''),('QUAT','Quaternion','')], default='EULER')
    sign_r: bpy.props.EnumProperty(name="Right Wheel Direction", items=[('PLUS','Forward (+1)',''),('MINUS','Inverted (-1)','')], default='PLUS')
    sign_l: bpy.props.EnumProperty(name="Left Wheel Direction", items=[('PLUS','Forward (+1)',''),('MINUS','Inverted (-1)','')], default='PLUS')

    wheel_forward_invert: bpy.props.BoolProperty(
        name="Invert Wheel Forward",
        description="Flip wheel rolling direction relative to body forward (use if the model's front faces the opposite axis)",
        default=False,
    )

    # Feasibility / Autocorrect
    body_forward_axis: bpy.props.EnumProperty(
        name="Body Forward Axis",
        items=[('+X','Local +X',''), ('-X','Local -X',''), ('+Y','Local +Y',''), ('-Y','Local -Y','')],
        default='+Y'
    )
    side_tol: bpy.props.FloatProperty(name="Sideways Tolerance (m/s)", default=0.02, min=0.0, precision=6)

    autocorrect_mode: bpy.props.EnumProperty(
        name="Autocorrect Mode (Path Geometry)",
        items=[
            ('OFF','Off',''),
            ('SEASE','Smooth Curve (S-Ease)','Smooth Bezier segments with curvature clamp'),
            ('LINEAR','Linear (Rotate–Move–Rotate)','Rotate to face, move straight, rotate to final'),
        ],
        default='SEASE',
    )
    bezier_tangent_scale: bpy.props.FloatProperty(
        name="Tangent Scale (S-Ease)",
        description="Handle length as fraction of pose distance (larger = rounder arcs). Curvature is clamped by track_width/2.",
        default=0.35, min=0.05, max=2.0, precision=3
    )
    linear_rotation_fraction: bpy.props.FloatProperty(
        name="Rotation Fraction (Linear)",
        description="Per segment, fraction of frames used for the initial and final rotations (each).",
        default=0.25, min=0.0, max=0.45, precision=3
    )

    # Speed profiles
    speed_profile: bpy.props.EnumProperty(
        name="Speed Profile (Timing)",
        items=[
            ('CONSTANT',     "Constant (Uniform+Ramps)",    "Constant speed with ramps (linear accel/decel)"),
            ('GLOBAL_EASE',  "Ease — Whole Timeline",       "Slow-in/fast/slow-out across the entire timeline"),
            ('PER_KEY_EASE', "Ease — Per Keyframe Segment", "Each keyframe-to-keyframe segment eases (single slider)"),
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

    # UI foldouts
    show_instructions: bpy.props.BoolProperty(name="Show Instructions", default=False)
    show_selection:   bpy.props.BoolProperty(name="Show Object Selection", default=True)
    show_calibration: bpy.props.BoolProperty(name="Show Calibration", default=False)
    show_csv_import:  bpy.props.BoolProperty(name="Show Import Animation by CSV", default=False)
    show_feasibility: bpy.props.BoolProperty(name="Show Feasibility", default=True)
    show_rpm_calc:    bpy.props.BoolProperty(name="Show RPM Calculation", default=False)
    show_anim_export: bpy.props.BoolProperty(name="Show Animation Data Export", default=False)
    show_csv_export:  bpy.props.BoolProperty(name="Show CSV Engineering Export", default=False)

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
    bl_idname="segway.autocorrect_linear"; bl_label="Autocorrect & Bake (Linear: Rotate–Move–Rotate)"
    def execute(self, context):
        P=context.scene.sg_props
        if P.autocorrect_mode!='LINEAR': self.report({'ERROR'},"Set Autocorrect Mode to 'Linear (Rotate–Move–Rotate)'."); return {'CANCELLED'}
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

class SG_OT_AttachDrivers(bpy.types.Operator):
    bl_idname="segway.attach_drivers"; bl_label="Attach Drivers"
    def execute(self, context):
        P=context.scene.sg_props
        if not bpy.app.driver_namespace.get(_driver_key()):
            self.report({'ERROR'}, "Build Cache first (and pass validation)."); return {'CANCELLED'}
        axis_i=_AXIS_INDEX[P.wheel_axis]
        for side in ('L','R'):
            for obj in _iter_side(P, side):
                if P.rotation_mode=='EULER':
                    _ensure_xyz_euler(obj)
                    try:
                        for i in range(4): obj.driver_remove("rotation_quaternion", i)
                    except Exception: pass
                    try: obj.driver_remove("rotation_euler", axis_i)
                    except Exception: pass
                    dcurve=obj.driver_add("rotation_euler",axis_i); dcurve.driver.type='SCRIPTED'
                    dcurve.driver.expression=f"sg_theta('{side}', frame)"
                else:
                    _ensure_quaternion(obj)
                    try: obj.driver_remove("rotation_euler", axis_i)
                    except Exception: pass
                    try:
                        for i in range(4): obj.driver_remove("rotation_quaternion", i)
                    except Exception: pass
                    for comp in range(4):
                        dcurve=obj.driver_add("rotation_quaternion",comp); dcurve.driver.type='SCRIPTED'
                        rq=_obj_rest_quat(obj)
                        obj['rest_w'], obj['rest_x'], obj['rest_y'], obj['rest_z'] = rq
                        dcurve.driver.expression=f"sg_quat_comp_obj('{side}', frame, {comp}, '{P.wheel_axis}', rw, rx, ry, rz)"
                        var=dcurve.driver.variables.new(); var.name='rw'; var.targets[0].id=obj; var.targets[0].data_path='["rest_w"]'
                        var=dcurve.driver.variables.new(); var.name='rx'; var.targets[0].id=obj; var.targets[0].data_path='["rest_x"]'
                        var=dcurve.driver.variables.new(); var.name='ry'; var.targets[0].id=obj; var.targets[0].data_path='["rest_y"]'
                        var=dcurve.driver.variables.new(); var.name='rz'; var.targets[0].id=obj; var.targets[0].data_path='["rest_z"]'
        self.report({'INFO'}, "Drivers attached to wheel objects."); return {'FINISHED'}

class SG_OT_BuildCache(bpy.types.Operator):
    bl_idname="segway.build_cache"; bl_label="Build Cache"
    def execute(self, context):
        try: d=build_cache(context)
        except Exception as e: self.report({'ERROR'}, str(e)); return {'CANCELLED'}
        self.report({'INFO'}, f"OK | r={d['radius']:.4f} m | track={d['track']:.4f} m | maxRPM L/R {d['max_rpm_L']:.1f}/{d['max_rpm_R']:.1f}")
        return {'FINISHED'}

class SG_OT_Bake(bpy.types.Operator):
    bl_idname="segway.bake_wheels"; bl_label="Bake Wheels"
    def execute(self, context):
        P=context.scene.sg_props; d=bpy.app.driver_namespace.get(_driver_key())
        if not d: self.report({'ERROR'},"Build Cache first (and pass validation)."); return {'CANCELLED'}
        f0=d['f0']; f1=f0+len(d['thetaL'])-1
        axis_i=_AXIS_INDEX[P.wheel_axis]; ux,uy,uz=_axis_unit(P.wheel_axis)
        for side in ('L','R'):
            arr=d['thetaL'] if side=='L' else d['thetaR']
            rest=d['restL'] if side=='L' else d['restR']
            for obj in _iter_side(P,side):
                if P.rotation_mode=='EULER':
                    _ensure_xyz_euler(obj)
                    try: obj.driver_remove("rotation_euler",axis_i)
                    except Exception: pass
                else:
                    _ensure_quaternion(obj)
                    try:
                        for i in range(4): obj.driver_remove("rotation_quaternion", i)
                    except Exception: pass
                for f in range(f0, f1+1):
                    i=f-f0; th=arr[i]
                    if P.rotation_mode=='EULER':
                        obj.rotation_euler[axis_i]=th
                        obj.keyframe_insert("rotation_euler", frame=f, index=axis_i)
                    else:
                        half=0.5*th; s=sin(half); c=cos(half)
                        q_spin=(c,ux*s,uy*s,uz*s)
                        aw,ax,ay,az=rest; bw,bx,by,bz=q_spin
                        q=(aw*bw-ax*bx-ay*by-az*bz,
                           aw*bx+ax*bw+ay*bz-az*by,
                           aw*by-ax*bz+ay*bw+az*bx,
                           aw*bz+ax*by-ay*bx+az*bw)
                        rq=obj.rotation_quaternion
                        rq[0],rq[1],rq[2],rq[3]=q
                        for comp in range(4): obj.keyframe_insert("rotation_quaternion", frame=f, index=comp)
        self.report({'INFO'}, f"Baked {f0}..{f1} to keyframes."); return {'FINISHED'}

class SG_OT_Clear(bpy.types.Operator):
    bl_idname="segway.clear"; bl_label="Clear Drivers/Keyframes"
    bl_description="Remove wheel rotation drivers AND their rotation keyframes (Euler & Quaternion) on all wheel objects."
    def execute(self, context):
        P=context.scene.sg_props; axis_i=_AXIS_INDEX[P.wheel_axis]
        removed_any=False
        for side in ('L','R'):
            for obj in _iter_side(P, side):
                try: obj.driver_remove("rotation_euler", axis_i); removed_any=True
                except Exception: pass
                try:
                    for i in range(4): obj.driver_remove("rotation_quaternion", i); removed_any=True
                except Exception: pass
                ad=obj.animation_data
                if ad and ad.action:
                    to_del=[fc for fc in ad.action.fcurves
                            if (fc.data_path=="rotation_euler" and fc.array_index==axis_i)
                            or fc.data_path=="rotation_quaternion"]
                    for fc in to_del:
                        try: ad.action.fcurves.remove(fc); removed_any=True
                        except Exception: pass
                if P.rotation_mode=='EULER':
                    try: obj.rotation_euler[axis_i]=0.0
                    except Exception: pass
                else:
                    try:
                        rq=obj.rotation_quaternion
                        rq[0],rq[1],rq[2],rq[3]=1.0,0.0,0.0,0.0
                    except Exception: pass
        self.report({'INFO'} if removed_any else {'WARNING'},
                    "Cleared drivers & rotation keyframes" if removed_any else "Nothing to clear on wheel objects.")
        return {'FINISHED'}

# ---------------------- CSV Export ----------------------
class SG_OT_ExportCSV(bpy.types.Operator):
    bl_idname="segway.export_csv"; bl_label="Export CSV"
    bl_description="Export t, x, y, yaw, thetaR/L, rateR/L using chosen sampling and units"
    def execute(self, context):
        P=context.scene.sg_props; d=bpy.app.driver_namespace.get(_driver_key())
        if not d: self.report({'ERROR'}, "Build Cache first (and pass validation)."); return {'CANCELLED'}
        path=bpy.path.abspath(P.csv_path)
        fps=d['fps']; n=len(d['thetaL'])
        ang_k=(1.0 if P.angle_unit=='RAD' else 180.0/pi)
        if   P.angrate_unit=='RPM': rate_k=60.0/(2.0*pi)
        elif P.angrate_unit=='RPS': rate_k=1.0/(2.0*pi)
        else:                        rate_k=180.0/pi
        len_k=(1.0 if P.length_unit=='M' else 100.0)
        rows=[]
        if P.sample_mode=='FRAME':
            for i in range(n):
                t=i/fps
                thL=d['thetaL'][i]; thR=d['thetaR'][i]
                x=d['x'][i]*len_k; y=d['y'][i]*len_k
                yaw=d['yaw'][i]*(180.0/pi if P.angle_unit=='DEG' else 1.0)
                if i==0: rateR=rateL=0.0
                else:
                    dthR=d['thetaR'][i]-d['thetaR'][i-1]
                    dthL=d['thetaL'][i]-d['thetaL'][i-1]
                    rateR=(dthR*fps)*rate_k; rateL=(dthL*fps)*rate_k
                rows.append((t,x,y,yaw, thR*ang_k, thL*ang_k, rateR, rateL))
        else:
            try: hz=float(P.fixed_rate)
            except: hz=100.0
            hz=max(1.0,hz); dt=1.0/hz; total=(n-1)/fps; k=0
            while True:
                t=k*dt
                if t>total+1e-9: break
                idx=t*fps
                thR=_linerp(d['thetaR'],idx); thL=_linerp(d['thetaL'],idx)
                x=_linerp(d['x'],idx)*len_k; y=_linerp(d['y'],idx)*len_k
                yaw=_linerp(d['yaw'],idx)*(180.0/pi if P.angle_unit=='DEG' else 1.0)
                if k==0: rateR=rateL=0.0
                else:
                    thRprev=rows[-1][4]/ang_k; thLprev=rows[-1][5]/ang_k
                    rateR=((thR-thRprev)/dt)*rate_k; rateL=((thL-thLprev)/dt)*rate_k
                rows.append((t,x,y,yaw, thR*ang_k, thL*ang_k, rateR, rateL))
                k+=1
        try:
            with open(path,"w",encoding="utf-8") as f:
                f.write("# Track width is center-to-center of left/right wheels.\n")
                f.write(f"# track_width_m={P.track_width:.6f}, tire_spacing_m={P.tire_spacing:.6f}, swap_lr={P.swap_lr}, wheel_forward_invert={P.wheel_forward_invert}\n")
                f.write(f"t,{('x_m' if P.length_unit=='M' else 'x_cm')},{('y_m' if P.length_unit=='M' else 'y_cm')},yaw_{P.angle_unit.lower()},thetaR_{P.angle_unit.lower()},thetaL_{P.angle_unit.lower()},rateR_{P.angrate_unit.lower()},rateL_{P.angrate_unit.lower()}\n")
                for r in rows:
                    f.write("{:.6f},{:.6f},{:.6f},{:.9f},{:.9f},{:.9f},{:.4f},{:.4f}\n".format(*r))
        except Exception as e:
            self.report({'ERROR'}, f"Failed to write CSV: {e}"); return {'CANCELLED'}
        self.report({'INFO'}, f"Wrote {path} ({len(rows)} samples)"); return {'FINISHED'}

# ---------------------- CSV Import ----------------------
class SG_OT_ImportCSV(bpy.types.Operator):
    bl_idname="segway.import_csv"; bl_label="Import Animation from CSV"
    bl_description="Import animation data from CSV file and apply to selected objects"
    
    def execute(self, context):
        P=context.scene.sg_props
        ch=P.chassis
        
        if not ch:
            self.report({'ERROR'}, "Assign the Chassis first."); return {'CANCELLED'}
        
        # Get selected objects for wheel collections
        colL=_col_for_side(P,'L'); colR=_col_for_side(P,'R')
        if not colL or not colR:
            self.report({'ERROR'}, "Assign Left and Right wheel collections first."); return {'CANCELLED'}
        
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
            
            # Debug: Show what we're working with
            self.report({'INFO'}, f"CSV has {len(rows)} total rows, skipping {skip_rows}, using {len(data_rows)} data rows")
            self.report({'INFO'}, f"Header row: {header}")
            self.report({'INFO'}, f"First few data rows: {data_rows[:3] if len(data_rows) >= 3 else data_rows}")
            
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
            
            # Debug: Show what columns we found
            self.report({'INFO'}, f"Found columns: {col_indices}")
            
            # Check required columns - your CSV format has t, x_m, y_m, yaw_rad
            required_cols = ['frame', 'x', 'y']  # z is optional, yaw_rad is rotation
            missing_cols = [col for col in required_cols if col not in col_indices]
            if missing_cols:
                self.report({'ERROR'}, f"Missing required columns: {missing_cols}")
                self.report({'ERROR'}, f"Available columns: {[header[i] for i in range(len(header))]}")
                self.report({'ERROR'}, f"Try adjusting 'Skip Rows from Top' setting (currently {skip_rows})")
                return {'CANCELLED'}
            
            # Clear existing animation on chassis
            if ch.animation_data and ch.animation_data.action:
                for fc in list(ch.animation_data.action.fcurves):
                    if fc.data_path in ("location", "rotation_euler", "rotation_quaternion"):
                        try: ch.animation_data.action.fcurves.remove(fc)
                        except Exception: pass
            
            # Create new action if needed
            if not ch.animation_data:
                ch.animation_data_create()
            if not ch.animation_data.action:
                ch.animation_data.action = bpy.data.actions.new(name="ImportedAnimation")
            
            # Process data rows
            frames_processed = 0
            for row in data_rows:
                if len(row) <= max(col_indices.values()):
                    continue  # Skip incomplete rows
                
                try:
                    # Parse frame number (t column in your CSV contains time in seconds)
                    time_seconds = float(row[col_indices['frame']])
                    # Convert time to frame number (assuming 30 FPS, adjust if needed)
                    fps = 30.0  # You can make this configurable later
                    frame = int(round(time_seconds * fps)) + 1  # +1 to start from frame 1
                    
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
            
            # Apply tire rotation to wheel objects if thetaR/thetaL columns exist
            if 'thetaR' in col_indices and 'thetaL' in col_indices:
                axis_i = _AXIS_INDEX[P.wheel_axis]
                ux, uy, uz = _axis_unit(P.wheel_axis)
                
                # Clear existing wheel animation
                for side in ('L', 'R'):
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
                        
                        # Clear keyframes
                        ad = obj.animation_data
                        if ad and ad.action:
                            to_del = [fc for fc in ad.action.fcurves
                                    if (fc.data_path == "rotation_euler" and fc.array_index == axis_i)
                                    or fc.data_path == "rotation_quaternion"]
                            for fc in to_del:
                                try: ad.action.fcurves.remove(fc)
                                except Exception: pass
                
                # Apply tire rotations
                for row in data_rows:
                    if len(row) <= max(col_indices.values()):
                        continue
                    
                    try:
                        # Convert time to frame number (same logic as above)
                        time_seconds = float(row[col_indices['frame']])
                        fps = 30.0
                        frame = int(round(time_seconds * fps)) + 1
                        thetaR = float(row[col_indices['thetaR']])
                        thetaL = float(row[col_indices['thetaL']])
                        
                        # Apply to right wheels
                        for obj in _iter_side(P, 'R'):
                            if P.rotation_mode == 'EULER':
                                obj.rotation_euler[axis_i] = thetaR
                                obj.keyframe_insert("rotation_euler", frame=frame, index=axis_i)
                            else:
                                # Quaternion rotation
                                half = 0.5 * thetaR
                                s = sin(half); c = cos(half)
                                q_spin = (c, ux*s, uy*s, uz*s)
                                rest = _obj_rest_quat(obj)
                                aw, ax, ay, az = rest
                                bw, bx, by, bz = q_spin
                                q = (aw*bw-ax*bx-ay*by-az*bz,
                                     aw*bx+ax*bw+ay*bz-az*by,
                                     aw*by-ax*bz+ay*bw+az*bx,
                                     aw*bz+ax*by-ay*bx+az*bw)
                                rq = obj.rotation_quaternion
                                rq[0], rq[1], rq[2], rq[3] = q
                                for comp in range(4):
                                    obj.keyframe_insert("rotation_quaternion", frame=frame, index=comp)
                        
                        # Apply to left wheels
                        for obj in _iter_side(P, 'L'):
                            if P.rotation_mode == 'EULER':
                                obj.rotation_euler[axis_i] = thetaL
                                obj.keyframe_insert("rotation_euler", frame=frame, index=axis_i)
                            else:
                                # Quaternion rotation
                                half = 0.5 * thetaL
                                s = sin(half); c = cos(half)
                                q_spin = (c, ux*s, uy*s, uz*s)
                                rest = _obj_rest_quat(obj)
                                aw, ax, ay, az = rest
                                bw, bx, by, bz = q_spin
                                q = (aw*bw-ax*bx-ay*by-az*bz,
                                     aw*bx+ax*bw+ay*bz-az*by,
                                     aw*by-ax*bz+ay*bw+az*bx,
                                     aw*bz+ax*by-ay*bx+az*bw)
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
        for fc in ad.action.fcurves:
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
                out={"meta":{"angle_unit":P.other_angle_unit,"fps":fps,"swap_lr":P.swap_lr,"wheel_forward_invert":P.wheel_forward_invert,
                             "frame_start":int(scn.frame_start),"frame_end":int(scn.frame_end)},
                     "samples":rows}
                with open(path,"w",encoding="utf-8") as f: json.dump(out,f,indent=2)
        except Exception as e:
            self.report({'ERROR'}, f"Failed to write: {e}"); return {'CANCELLED'}
        self.report({'INFO'}, f"Keyframes exported to {path} ({len(rows)} rows)"); return {'FINISHED'}

# ---------------------- UI ----------------------
class SG_PT_Panel(bpy.types.Panel):
    bl_label="True RoboAnimator"
    bl_space_type='VIEW_3D'
    bl_region_type='UI'
    bl_category="True RoboAnimator"

    def _section(self, layout, prop_flag, title):
        box=layout.box(); header=box.row(); P=bpy.context.scene.sg_props
        is_open=getattr(P,prop_flag); icon='TRIA_DOWN' if is_open else 'TRIA_RIGHT'
        header.prop(P, prop_flag, text="", icon=icon, emboss=False)
        header.label(text=title)
        return box if is_open else None

    def draw(self, context):
        layout=self.layout; P=context.scene.sg_props
        layout.use_property_split=True; layout.use_property_decorate=False

        box = self._section(layout, "show_instructions", "Instructions")
        if box:
            c = box.column(align=True)
            c.label(text="1) Animate chassis (location + rotation).")
            c.label(text="2) Validate Motion (no sideways slip).")
            c.label(text="3) Autocorrect: S-Curve or Linear.")
            c.label(text="4) Pick Speed Profile (Constant+ramps / Ease Whole / Per Segment).")
            c.label(text="5) Build Cache → Attach Drivers (or Bake Wheels).")
            c.label(text="6) Export (keyframes / CSV).")
            c.label(text="Tip: If wheels roll backwards, toggle Wheel Forward Invert or swap L/R.")

        box=self._section(layout, "show_selection", "Object Selection")
        if box:
            col=box.column(align=True)
            col.prop(P,"chassis")
            col.prop(P,"right_collection")
            col.prop(P,"left_collection")
            col.prop(P,"swap_lr")

        box=self._section(layout, "show_calibration", "Calibration & Wheel Setup")
        if box:
            col=box.column(align=True)
            col.label(text="Geometry (meters)")
            col.prop(P,"track_width")
            col.prop(P,"tire_spacing")
            col.prop(P,"auto_radius")
            if not P.auto_radius: col.prop(P,"wheel_radius")
            col.separator(); col.label(text="Wheel Rotation")
            col.prop(P,"wheel_axis"); col.prop(P,"rotation_mode")
            col.prop(P,"sign_r"); col.prop(P,"sign_l")
            col.prop(P,"wheel_forward_invert")

        box=self._section(layout, "show_csv_import", "Import Animation by CSV")
        if box:
            col=box.column(align=True)
            col.prop(P,"csv_import_path")
            col.prop(P,"csv_import_skip_rows")
            col.separator()
            col.label(text="Imports chassis position/rotation and tire rotation")
            col.label(text="from CSV with columns: frame, x, y, z, thetaR, thetaL")
            col.label(text="(and optionally euler_x/y/z or quat_w/x/y/z)")
            box.operator("segway.import_csv", icon='IMPORT')

        box=self._section(layout, "show_feasibility", "Feasibility & Autocorrect")
        if box:
            col=box.column(align=True)
            col.prop(P, "body_forward_axis")
            col.prop(P, "side_tol")
            col.prop(P, "autocorrect_mode")
            col.prop(P, "bezier_tangent_scale")
            col.prop(P, "linear_rotation_fraction")
            col.prop(P, "speed_profile")
            rC=col.row(align=True); rC.enabled=(P.speed_profile=='CONSTANT'); rC.prop(P,"constant_ramp_frames")
            r=col.row(align=True); r.enabled=(P.speed_profile=='GLOBAL_EASE'); r.prop(P,"timeline_ease_frames")
            rS=col.row(align=True); rS.enabled=(P.speed_profile=='PER_KEY_EASE'); rS.prop(P,"segment_ease_frames")
            row=box.row(align=True)
            row.operator("segway.validate_motion", icon='INFO')
            row.operator("segway.autocorrect_bake", icon='MOD_CURVE')
            box.operator("segway.revert_autocorrect", icon='LOOP_BACK')

        box=self._section(layout, "show_rpm_calc", "RPM Calculation & Limits")
        if box:
            lim=box.box().column(align=True)
            lim.prop(P,"max_rpm")
            lim.prop(P,"max_ang_accel_rpm_s")
            row=box.row(align=True)
            row.operator("segway.build_cache", icon='IMPORT')
            row.operator("segway.attach_drivers", icon='CONSTRAINT')
            row=box.row(align=True)
            row.operator("segway.bake_wheels", icon='REC')
            row.operator("segway.clear", icon='TRASH')

        box=self._section(layout, "show_anim_export", "Animation Data Export (Keyframes)")
        if box:
            f=box.box(); f.prop(P,"other_export_path")
            o=box.box(); o.prop(P,"other_export_format"); o.prop(P,"other_angle_unit")
            r=box.box(); r.operator("segway.export_keyframes", icon='EXPORT')

        box=self._section(layout, "show_csv_export", "CSV Engineering Export")
        if box:
            f=box.box(); f.prop(P,"csv_path")
            opts=box.box().column(align=True)
            opts.prop(P,"sample_mode")
            if P.sample_mode=='FIXED': opts.prop(P,"fixed_rate")
            opts.prop(P,"angle_unit"); opts.prop(P,"angrate_unit"); opts.prop(P,"length_unit")
            run=box.box(); run.operator("segway.export_csv", icon='EXPORT')

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
    for c in classes: bpy.utils.register_class(c)
    bpy.types.Scene.sg_props=bpy.props.PointerProperty(type=SG_Props)
    bpy.app.driver_namespace['sg_theta']=sg_theta
    bpy.app.driver_namespace['sg_quat_comp']=sg_quat_comp
    bpy.app.driver_namespace['sg_quat_comp_obj']=sg_quat_comp_obj

def unregister():
    for c in reversed(classes): bpy.utils.unregister_class(c)
    for k in ('sg_theta','sg_quat_comp','sg_quat_comp_obj', _driver_key()):
        bpy.app.driver_namespace.pop(k, None)
    del bpy.types.Scene.sg_props

if __name__=="__main__":
    register()
