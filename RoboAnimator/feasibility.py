import bpy
from math import hypot

def _sample(ctx):
    p = ctx.scene.ra_props
    obj = p.chassis
    if not obj:
        return []
    fs = range(ctx.scene.frame_start, ctx.scene.frame_end + 1)
    out = []
    for f in fs:
        bpy.context.scene.frame_set(f)
        mw = obj.matrix_world
        x, y = mw.translation.x, mw.translation.y
        out.append((f, x, y))
    return out

def check(ctx):
    pts = _sample(ctx)
    if len(pts) < 2:
        return {"ok": False, "msg": "No chassis keys."}
    d = 0.0
    for i in range(1, len(pts)):
        _, x0, y0 = pts[i-1]
        _, x1, y1 = pts[i]
        d += hypot(x1 - x0, y1 - y0)
    return {"ok": True, "msg": f"Frames {len(pts)}  Path ≈ {d:.3f} m"}
