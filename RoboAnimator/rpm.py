import bpy
from math import pi

CACHE_KEY = "TRA_CACHE"

def build(ctx):
    """Zeroed cache for now. Limits stored for future checks."""
    scn = ctx.scene
    p = scn.ra_props
    f0, f1 = scn.frame_start, scn.frame_end
    n = (f1 - f0) + 1
    cache = {
        "f0": f0,
        "thetaL": [0.0]*n,
        "thetaR": [0.0]*n,
        "r": p.wheel_r_m,
        "track": p.track_m,
        "max_rpm": p.max_rpm,
        "max_rpm_s": p.max_rpm_s,
    }
    ns = bpy.app.driver_namespace
    ns[CACHE_KEY] = cache
    if "sg_theta" not in ns:
        def sg_theta(side, frame):
            d = bpy.app.driver_namespace.get(CACHE_KEY) or {}
            i = int(frame - d.get("f0", 0))
            arr = d.get("thetaL" if side == "L" else "thetaR", [])
            if i < 0 or i >= len(arr):
                return 0.0
            return arr[i]
        ns["sg_theta"] = sg_theta
    return cache

def rpm(dth_now, dth_prev, fps):
    dth = dth_now - dth_prev
    rev = dth / (2 * pi)
    return rev * fps * 60.0

def add_drivers(ctx):
    """Attach one rotation driver on the selected axis. Remove others."""
    scn = ctx.scene
    p = scn.ra_props
    axis = p.wheel_axis
    axis_idx = {'X':0,'Y':1,'Z':2}[axis]
    ns = bpy.app.driver_namespace
    ns.setdefault("sg_theta", lambda side, f: 0.0)

    def _apply(col, side):
        if not col:
            return
        for o in col.objects:
            # clear existing rotation drivers
            ad = o.animation_data
            if ad and ad.drivers:
                for fc in list(ad.drivers):
                    if fc.data_path == "rotation_euler":
                        o.driver_remove("rotation_euler", fc.array_index)
            # add new driver on chosen axis
            fcurve = o.driver_add("rotation_euler", axis_idx)
            d = fcurve.driver
            d.type = 'SCRIPTED'
            v = d.variables.new()
            v.name = "f"
            v.type = 'SINGLE_PROP'
            v.targets[0].id_type = 'SCENE'
            v.targets[0].id = scn
            v.targets[0].data_path = "frame_current"
            d.expression = f"sg_theta('{side}', f)"
            # lock other axes
            o.lock_rotation[0] = (axis_idx != 0)
            o.lock_rotation[1] = (axis_idx != 1)
            o.lock_rotation[2] = (axis_idx != 2)

    _apply(p.wheels_l, "L")
    _apply(p.wheels_r, "R")
