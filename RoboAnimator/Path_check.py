import bpy, bisect
from math import pi, sin, cos, sqrt, atan2

from .utils import (
    _backup_chassis_keys, _restore_chassis_keys,
    _bezier_point_xy, _bezier_tangent_xy, _bezier_eval,
    _build_arc_lut_norm_total_xy, _arc_to_t_from_lut,
    _trapezoid_s, _edge_ease_progress, _edge_ease_progress_asym, _ease_in_out_cubic,
    yaw_to_heading, heading_to_yaw,
    _body_basis_from_yaw, _unwrap, _wrap, _linerp, _lerp, _ensure_xyz_euler,
    _get_chassis_key_poses,
)
# from .kinematics_rpm import build_cache  # not used

# --- missing in your split: bake utility used by both builders ---
def _bake_chassis_frames_from_heading_samples(P, ch, samples):
    _ensure_xyz_euler(ch)
    scn = bpy.context.scene
    deps = bpy.context.evaluated_depsgraph_get()

    ad = ch.animation_data
    if ad and ad.action:
        for fc in list(ad.action.fcurves):
            if fc.data_path in ("location", "rotation_euler"):
                try:
                    ad.action.fcurves.remove(fc)
                except Exception:
                    pass

    for f, x, y, z, h in samples:
        ch.location.x = x
        ch.location.y = y
        ch.location.z = z
        ch.rotation_euler[0] = 0.0
        ch.rotation_euler[1] = 0.0
        ch.rotation_euler[2] = heading_to_yaw(P, h)
        ch.keyframe_insert("location", frame=f, index=-1)
        ch.keyframe_insert("rotation_euler", frame=f, index=-1)

    deps.update()
    scn.frame_set(scn.frame_start)
    deps.update()

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

    baked=[]; last_h=0.0
    cum_frames=[]; cum_len=[]; acc_f=0.0; acc_L=0.0
    for i,seg in enumerate(segs):
        acc_f+=spans[i]; acc_L+=seg["L"]
        cum_frames.append(acc_f); cum_len.append(acc_L)

    for f in range(f0, f1+1):
        tau=(f-f0)/max(1,total_frames)

        if profile=='CONSTANT':
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
            span_f=max(1, int(round(cum_frames[idx]-fr_prev)))
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
    CONSTANT: global trapezoid arc-length mapping for uniform linear speed.
    GLOBAL_EASE / PER_KEY_EASE: segment-local timing.
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
    f_ramp = (const_ramp_frames/total_frames) if total_frames>0 else 0.0

    _backup_chassis_keys(ch)
    poses=_get_chassis_key_poses(P,ch)
    if len(poses)<2: raise RuntimeError("Need at least two keyed poses on the chassis (location/rotation).")

    segs=[]; spans=[]; total_len=0.0
    for i in range(len(poses)-1):
        fA,xA,yA,zA,_yA,hA=poses[i]
        fB,xB,yB,zB,_yB,hB=poses[i+1]
        spans.append(max(1, fB-fA))
        dx=xB-xA; dy=yB-yA
        L=sqrt(dx*dx+dy*dy)
        if L<=1e-12:
            segs.append({"mode":"rotonly","fA":fA,"fB":fB,"xA":xA,"yA":yA,"zA":zA,"xB":xB,"yB":yB,"zB":zB,"hA":hA,"hB":hB,"L":0.0})
        else:
            segs.append({"mode":"move","fA":fA,"fB":fB,"xA":xA,"yA":yA,"zA":zA,"xB":xB,"yB":yB,"zB":zB,"hA":hA,"hB":hB,"L":L,"dir":atan2(dy,dx)})
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
        last_h=poses[0][5]
        for f in range(f0, f1+1):
            tau=(f-f0)/max(1,total_frames)
            s_norm=_trapezoid_s(tau, f_ramp)
            s_target=s_norm*total_len
            idx=bisect.bisect_left(cum_len, s_target)
            idx=min(max(idx,0), len(segs)-1)

            s_prev=0.0 if idx==0 else cum_len[idx-1]
            seg=segs[idx]

            if seg.get("L",0.0)<=1e-12:
                x,y,z=seg["xB"],seg["yB"],seg["zB"]; h=seg["hB"]
            else:
                u=0.0 if seg["L"]<=1e-12 else (s_target - s_prev)/seg["L"]
                u=min(max(u,0.0),1.0)
                x=_lerp(seg["xA"], seg["xB"], u)
                y=_lerp(seg["yA"], seg["yB"], u)
                z=_lerp(seg["zA"], seg["zB"], u)
                h=seg["dir"]
            baked.append((f, x, y, z, h)); last_h=h

    else:
        for i in range(len(poses)-1):
            fA,xA,yA,zA,_yA,hA=poses[i]
            fB,xB,yB,zB,_yB,hB=poses[i+1]
            span=max(1, fB-fA)
            dir_heading=atan2(yB-yA, xB-xA)

            n1=int(round(span*rot_frac)); n3=int(round(span*rot_frac)); n2=span-n1-n3
            if n2<0:
                shrink=-n2; s1=min(n1,(shrink+1)//2); s3=min(n3, shrink-s1)
                n1-=s1; n3-=s3; n2=0

            baked.append((fA,xA,yA,zA,hA)); idx=0

            if n1>0:
                for s in range(1,n1+1):
                    u=s/n1; t=_ease_in_out_cubic(u)
                    h=_angle_lerp(hA,dir_heading,t)
                    baked.append((fA+idx+s, xA,yA,zA,h))
                idx+=n1

            if n2>0:
                if profile=='GLOBAL_EASE':
                    fin=fout=max(0.0, min(0.49, getattr(P,"timeline_ease_frames",15)/max(1.0,n2)))
                    for s in range(1,n2+1):
                        u=s/n2; u=_edge_ease_progress_asym(u, fin, fout)
                        x=xA+u*(xB-xA); y=yA+u*(yB-yA); z = zA + u*(zB - zA)
                        baked.append((fA+idx+s, x,y,z,dir_heading))
                else:  # PER_KEY_EASE
                    fin=fout=seg_ease_frames/max(1.0,n2)
                    for s in range(1,n2+1):
                        u=s/n2; u=_edge_ease_progress_asym(u, fin, fout)
                        x=xA+u*(xB-xA); y=yA+u*(yB-yA); z = zA + u*(zB - zA)
                        baked.append((fA+idx+s, x,y,z,dir_heading))
                idx+=n2

            if n3>0:
                h_start=dir_heading
                for s in range(1,n3+1):
                    u=s/n3; t=_ease_in_out_cubic(u)
                    h=_angle_lerp(h_start,hB,t)
                    baked.append((fA+idx+s, xB,yB,zB,h))
                idx+=n3

            baked.append((fB,xB,yB,zB,hB))

    byf={fr:(fr,x,y,z,h) for (fr,x,y,z,h) in baked}
    out=[byf[k] for k in sorted(byf.keys())]
    _bake_chassis_frames_from_heading_samples(P,ch,out)
    return len(out)
