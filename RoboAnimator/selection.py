import bpy

# ----- internal helpers -----
def _apply_axis_locks_from_props(p):
    """Lock rotation to p.wheel_axis on every wheel object."""
    idx = {'X':0,'Y':1,'Z':2}[p.wheel_axis]
    for col in (p.wheels_l, p.wheels_r):
        if not col:
            continue
        for o in col.objects:
            o.lock_rotation[0] = (idx != 0)
            o.lock_rotation[1] = (idx != 1)
            o.lock_rotation[2] = (idx != 2)

def _on_axis_update(self, context):
    _apply_axis_locks_from_props(self)

def _sorted_objs(col):
    return sorted(list(col.objects), key=lambda o: (o.name.lower(), o.as_pointer()))

def _even_in_bounds(n, lo=2, hi=8):
    n = max(lo, min(hi, int(n)))
    if n % 2:  # force even
        n = n + 1 if n < hi else n - 1
    return n

def _on_total_wheels_update(self, context):
    # Coerce to an even number within [2,8]
    fixed = _even_in_bounds(self.total_wheels)
    if fixed != self.total_wheels:
        self.total_wheels = fixed  # triggers no loops in Blender

# ----- public API -----
def P(ctx):
    return ctx.scene.ra_props

def list_wheels(ctx):
    """Return dict {'L':[objs...], 'R':[objs...]} with stable order."""
    p = P(ctx)
    L = _sorted_objs(p.wheels_l) if p.wheels_l else []
    R = _sorted_objs(p.wheels_r) if p.wheels_r else []
    return {'L': L, 'R': R}

def refresh_wheels(ctx):
    """
    Compute counts and mismatches:
      - total must equal desired even count
      - left count must equal right count
      - found total must be even
    """
    wheels = list_wheels(ctx)
    nL, nR = len(wheels['L']), len(wheels['R'])
    total_found = nL + nR
    want = _even_in_bounds(P(ctx).total_wheels)
    side_mismatch = (nL != nR)
    odd_found = (total_found % 2 != 0)
    mismatch = (total_found != want) or side_mismatch or odd_found
    return {
        'L': nL, 'R': nR,
        'total': total_found, 'want': want,
        'side_mismatch': side_mismatch,
        'odd_found': odd_found,
        'mismatch': mismatch,
    }

def _pick_wheel_ref(ctx):
    wheels = list_wheels(ctx)
    if wheels['L']:
        return wheels['L'][0]
    if wheels['R']:
        return wheels['R'][0]
    return None

def auto_radius(ctx):
    """Detect radius: closest pair among bbox dims → diameter; radius = half."""
    ref = _pick_wheel_ref(ctx)
    if not ref:
        raise RuntimeError("Put at least one wheel object into Left or Right collection.")
    d = ref.dimensions
    vals = sorted([abs(d.x), abs(d.y), abs(d.z)])
    a, b, c = vals
    d1, d2 = (a, b) if abs(a - b) <= abs(b - c) else (b, c)
    if max(d1, d2) <= 1e-9:
        raise RuntimeError("Wheel dimensions too small.")
    diameter = max(d1, d2) if abs(d1 - d2) / max(d1, d2) > 0.01 else 0.5 * (d1 + d2)
    P(ctx).wheel_r_m = float(diameter) * 0.5
    return P(ctx).wheel_r_m

# ----- properties -----
class Props(bpy.types.PropertyGroup):
    # total wheels (forced even in [2,8])
    total_wheels: bpy.props.IntProperty(
        name="Total Wheels",
        description="Expected total wheel objects across Left + Right (even only)",
        default=2, min=2, max=8,
        update=_on_total_wheels_update,
    )

    # objects/collections
    chassis: bpy.props.PointerProperty(name="Chassis", type=bpy.types.Object)
    wheels_l: bpy.props.PointerProperty(name="Left Wheels", type=bpy.types.Collection)
    wheels_r: bpy.props.PointerProperty(name="Right Wheels", type=bpy.types.Collection)

    # sizes
    wheel_r_m: bpy.props.FloatProperty(name="Wheel Radius (m)", default=0.05, min=1e-5, precision=4)
    track_m:   bpy.props.FloatProperty(name="Track Width (m)", default=0.40, min=1e-5, precision=4)

    # limits
    max_rpm:   bpy.props.FloatProperty(name="Max Wheel Speed (RPM)",   default=0.0, min=0.0)
    max_rpm_s: bpy.props.FloatProperty(name="Max Wheel Accel (RPM/s)", default=0.0, min=0.0)

    # wheel rotation axis (changing locks rotation to that axis)
    wheel_axis: bpy.props.EnumProperty(
        name="Wheel Axis",
        items=[('X','X',''),('Y','Y',''),('Z','Z','')],
        default='X',
        update=_on_axis_update,
    )

    # ui
    show_instr:  bpy.props.BoolProperty(name="Show Instructions", default=True)
    show_selcal: bpy.props.BoolProperty(name="Selection and calibration", default=True)
    show_ops:    bpy.props.BoolProperty(name="Show Operations",  default=True)

def register_props():
    bpy.types.Scene.ra_props = bpy.props.PointerProperty(type=Props)

def unregister_props():
    if hasattr(bpy.types.Scene, "ra_props"):
        del bpy.types.Scene.ra_props
