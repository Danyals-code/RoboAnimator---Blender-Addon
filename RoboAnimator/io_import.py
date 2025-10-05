# io_import.py — Robust CSV importer for True RoboAnimator
# - Skips leading comment/blank lines (handles '# ...' headers)
# - Binds to calibrated chassis by default (Scene.sg_props.chassis)
# - Flexible header parsing: time / ms / frames; x/y in m or cm; yaw in deg or rad
# - Attaches CSV as Text datablock for provenance (optional)
# - Clears previous X/Y/Yaw fcurves (optional)
# - Keys LEFT/RIGHT wheel rotation from thetaL/thetaR (deg/rad), preserving base pose

import bpy
import csv
import io
import math
import re
from bpy_extras.io_utils import ImportHelper


# ------------- header / parsing helpers -------------

def _norm_token(s: str) -> str:
    """Normalize header token:
    - lowercase, strip spaces/underscores
    - remove content within [] or ()
    """
    s = (s or "").strip().lower()
    s = re.sub(r"\[[^\]]*\]|\([^\)]*\)", "", s)  # drop [units] and (units)
    s = s.replace(" ", "").replace("_", "")
    return s

def _find_time_col(header):
    """
    Find a time-like column.
    Returns: (index, mode) where mode in {'seconds','ms','frames'}; otherwise (-1, None).
    Accepts: t, time, time_s, time(s), timesec, timestamp, sec/seconds,
             ms/millis/milliseconds,
             frame/frames.
    """
    hn = [_norm_token(h) for h in header]
    candidates = {
        "t": "seconds",
        "time": "seconds",
        "times": "seconds",
        "timesec": "seconds",
        "timestamp": "seconds",
        "sec": "seconds",
        "second": "seconds",
        "seconds": "seconds",

        "ms": "ms",
        "millis": "ms",
        "milliseconds": "ms",

        "frame": "frames",
        "frames": "frames",
    }
    for i, tok in enumerate(hn):
        if tok in candidates:
            return i, candidates[tok]
    for i, tok in enumerate(hn):
        if tok.startswith("time"):
            return i, "seconds"
    return -1, None

def _find_col_by_names(header, names):
    """Find a column index by normalized names set."""
    hn = [_norm_token(h) for h in header]
    for i, tok in enumerate(hn):
        if tok in names:
            return i
    return -1

def _maybe_float(cell):
    try:
        return float(cell)
    except Exception:
        return None


# ------------- animation helpers -------------

def _seconds_to_frame(scene, t_s):
    fps = scene.render.fps / (scene.render.fps_base if scene.render.fps_base else 1.0)
    return round(t_s * fps)

def _ensure_text_from_csv(filepath):
    """Create (or reuse) a Text datablock from the CSV file for provenance."""
    # Blender 4.x: display_name_from_filepath
    name = f"CSV_{bpy.path.display_name_from_filepath(filepath)}"
    base = name
    i = 1
    while name in bpy.data.texts:
        i += 1
        name = f"{base}_{i}"
    txt = bpy.data.texts.new(name)
    # robust file read
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            txt.from_string(f.read())
    except Exception:
        with open(filepath, 'r') as f:
            txt.from_string(f.read())
    return txt

def _clear_existing_curves(obj):
    """Remove existing fcurves for location (x,y) and rotation_euler.z."""
    if not obj.animation_data or not obj.animation_data.action:
        return
    act = obj.animation_data.action
    to_remove = [fc for fc in act.fcurves if fc.data_path in ("location", "rotation_euler")]
    for fc in to_remove:
        act.fcurves.remove(fc)


# ------------- wheel helpers -------------

def _resolve_wheels_from_props(P):
    """Return (left_obj, right_obj) from various common property names."""
    if not P:
        return None, None
    cand_left = ["wheel_left", "left_wheel", "wheelL"]
    cand_right = ["wheel_right", "right_wheel", "wheelR"]
    L = R = None
    for n in cand_left:
        if hasattr(P, n) and getattr(P, n) is not None:
            L = getattr(P, n)
            break
    for n in cand_right:
        if hasattr(P, n) and getattr(P, n) is not None:
            R = getattr(P, n)
            break
    # ensure they are Objects
    if L and not isinstance(L, bpy.types.Object): L = None
    if R and not isinstance(R, bpy.types.Object): R = None
    return L, R

def _resolve_wheel_axis_index(P):
    """
    Get the wheel spin axis index from props if available.
    Accepts: 'X'|'Y'|'Z' or 0|1|2 in props named wheel_spin_axis or wheel_axis.
    Defaults to Y-axis (1).
    """
    idx = None
    for name in ("wheel_spin_axis", "wheel_axis"):
        if hasattr(P, name):
            val = getattr(P, name)
            if isinstance(val, str):
                up = val.strip().upper()
                if up in ("X", "Y", "Z"):
                    return {"X": 0, "Y": 1, "Z": 2}[up]
            else:
                try:
                    v = int(val)
                    if v in (0, 1, 2):
                        return v
                except Exception:
                    pass
    return 1  # default Y

def _record_base_rot(L, R):
    """Capture base Euler rotations to preserve initial pose."""
    baseL = tuple(L.rotation_euler) if L else None
    baseR = tuple(R.rotation_euler) if R else None
    return baseL, baseR

def _apply_wheel_angle(obj, axis_idx, base_euler, theta_rad):
    """Set rotation_euler on given axis to base + theta."""
    if not obj:
        return
    e = list(obj.rotation_euler)
    base = base_euler[axis_idx] if base_euler else 0.0
    e[axis_idx] = base + (theta_rad if theta_rad is not None else 0.0)
    obj.rotation_euler = e


# ------------- operator -------------

class SG_OT_ImportCSVAnim(bpy.types.Operator, ImportHelper):
    """Import robot path (x, y, yaw) from CSV and keyframe onto the calibrated chassis (plus wheel rotation)."""
    bl_idname = "segway.import_csv_anim"
    bl_label = "Import CSV Animation"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".csv"
    filter_glob: bpy.props.StringProperty(default="*.csv", options={'HIDDEN'})

    # Drive from calibrated object by default
    use_calibrated_object: bpy.props.BoolProperty(
        name="Use calibrated object (sg_props.chassis)",
        description="Attach animation to the object set in Object Selection & Calibration. "
                    "If off, falls back to the active object.",
        default=True
    )

    clear_previous_keys: bpy.props.BoolProperty(
        name="Clear previous animation for X/Y/Yaw",
        default=True
    )

    attach_csv_text: bpy.props.BoolProperty(
        name="Attach CSV as Text datablock (provenance)",
        default=True
    )

    def _read_csv_skip_comments(self, filepath):
        """Read CSV while skipping leading '#' comment lines and blank lines."""
        with open(filepath, "r", encoding="utf-8", errors="replace") as f:
            raw = f.read().splitlines()

        cleaned = []
        for line in raw:
            if not line:
                continue  # drop blanks anywhere
            s = line.lstrip()
            if s.startswith("#"):
                continue
            cleaned.append(line)

        if not cleaned:
            return []

        buf = io.StringIO("\n".join(cleaned))
        reader = csv.reader(buf)
        return list(reader)

    def execute(self, context):
        # ---------- resolve calibrated targets ----------
        P = getattr(context.scene, "sg_props", None)
        if self.use_calibrated_object and P and getattr(P, "chassis", None):
            obj = P.chassis
        else:
            obj = context.active_object

        if obj is None:
            self.report({'ERROR'}, "No target. Set ‘Chassis’ in Object Selection & Calibration or select an active object.")
            return {'CANCELLED'}

        # Resolve wheels (optional)
        wheelL, wheelR = _resolve_wheels_from_props(P)
        spin_axis = _resolve_wheel_axis_index(P)

        # ---------- read CSV (skip # comments / blanks) ----------
        rows = self._read_csv_skip_comments(self.filepath)
        if not rows or len(rows) < 2:
            self.report({'ERROR'}, "CSV seems empty or missing header (after skipping comments).")
            return {'CANCELLED'}

        header = rows[0]
        data = rows[1:]

        # ---------- detect columns ----------
        # time
        i_t, t_mode = _find_time_col(header)
        if i_t < 0:
            self.report(
                {'ERROR'},
                "No time column found. Acceptable: t, time, time(s), timestamp, sec/seconds, ms/millis, frame/frames."
            )
            self.report({'INFO'}, f"Detected header: {header}")
            return {'CANCELLED'}

        # x, y (meters or cm). Common robotics tokens: x_m, y_m -> normalized to xm/ym
        x_meter_names = {
            "x", "xm", "xmeter", "xmeters", "xmetre", "xmetres",
            "positionx", "posx", "xpos", "xposition"
        }
        y_meter_names = {
            "y", "ym", "ymeter", "ymeters", "ymetre", "ymetres",
            "positiony", "posy", "ypos", "yposition"
        }
        x_cm_names = {"xcm", "xcentimeter", "xcentimeters", "xcentimetre", "xcentimetres"}
        y_cm_names = {"ycm", "ycentimeter", "ycentimeters", "ycentimetre", "ycentimetres"}

        i_x_m = _find_col_by_names(header, x_meter_names)
        i_y_m = _find_col_by_names(header, y_meter_names)
        i_x_cm = _find_col_by_names(header, x_cm_names)
        i_y_cm = _find_col_by_names(header, y_cm_names)

        if (i_x_m < 0 and i_x_cm < 0) or (i_y_m < 0 and i_y_cm < 0):
            self.report({'ERROR'}, "No X/Y columns found. Expect x/y in meters or centimeters (headers can include units).")
            self.report({'INFO'}, f"Detected header: {header}")
            return {'CANCELLED'}

        # yaw (deg or rad) — also accept generic "yaw" as radians
        yaw_deg_names = {"yawdeg", "yawdegree", "yawdegrees", "headingdeg", "headingdegrees", "heading"}
        yaw_rad_names = {"yawrad", "yawradian", "yawradians"}
        generic_yaw_name = {"yaw"}

        i_yaw_deg = _find_col_by_names(header, yaw_deg_names)
        i_yaw_rad = _find_col_by_names(header, yaw_rad_names)
        i_yaw_generic = _find_col_by_names(header, generic_yaw_name)

        # wheel angles: thetaL / thetaR in rad/deg (from your sample CSV)
        # Normalization: thetaL_rad -> thetalrad; thetaR_rad -> thetarrad, etc.
        thetaL_rad_names = {"thetalrad", "leftthetarad", "thetaleft", "thetaleftinrad"}
        thetaR_rad_names = {"thetarrad", "rightthetarad", "thetarright", "thetarrightinrad"}
        thetaL_deg_names = {"thetaldeg", "leftthetadeg", "thetaleftdeg"}
        thetaR_deg_names = {"thetardeg", "rightthetadeg", "thetarrightdeg"}

        # explicitly include common exact headers
        thetaL_rad_names |= {"thet_l_rad", "thetaleft_rad", "thetalrad", "thet l rad"}
        thetaR_rad_names |= {"thet_r_rad", "thetaright_rad", "thetarrad", "thet r rad"}
        thetaL_deg_names |= {"thet_l_deg", "thetaleft_deg", "thetldeg", "thet l deg"}
        thetaR_deg_names |= {"thet_r_deg", "thetaright_deg", "thetardeg", "thet r deg"}

        # and very common: "thetaL_rad", "thetaR_rad", "thetaL_deg", "thetaR_deg"
        i_thetaL_rad = _find_col_by_names(header, {"thetalrad", "thetal", "thetalr"} | thetaL_rad_names)
        i_thetaR_rad = _find_col_by_names(header, {"thetarrad", "thetar", "thetarr"} | thetaR_rad_names)
        i_thetaL_deg = _find_col_by_names(header, {"thetaldeg"} | thetaL_deg_names)
        i_thetaR_deg = _find_col_by_names(header, {"thetardeg"} | thetaR_deg_names)

        # ---------- attach CSV text (optional) ----------
        if self.attach_csv_text:
            txt = _ensure_text_from_csv(self.filepath)
            obj["roboanim_csv_text"] = txt.name

        # ---------- clear previous ----------
        if self.clear_previous_keys:
            _clear_existing_curves(obj)
            # Note: we do NOT clear wheel fcurves; we simply key the chosen axis.

        # ---------- keyframe pass ----------
        scene = context.scene
        obj.animation_data_create()
        if wheelL: wheelL.animation_data_create()
        if wheelR: wheelR.animation_data_create()

        # Keep the starting wheel pose as base (so CSV angles add on top)
        baseL, baseR = _record_base_rot(wheelL, wheelR)

        any_keys = False

        # Precompute FPS for frame->time conversion
        fps = scene.render.fps / (scene.render.fps_base if scene.render.fps_base else 1.0)
        if fps <= 0:
            fps = 24.0  # sane fallback

        for r in data:
            if not r or len(r) <= i_t:
                continue

            # ---- time to seconds ----
            t_raw = _maybe_float(r[i_t])
            if t_raw is None:
                continue

            if t_mode == "seconds":
                t_s = t_raw
            elif t_mode == "ms":
                t_s = t_raw / 1000.0
            elif t_mode == "frames":
                t_s = t_raw / fps
            else:
                t_s = t_raw

            f = _seconds_to_frame(scene, t_s)

            # ---- X ----
            x_m = None
            if i_x_m >= 0 and len(r) > i_x_m:
                x_m = _maybe_float(r[i_x_m])
            elif i_x_cm >= 0 and len(r) > i_x_cm:
                v = _maybe_float(r[i_x_cm])
                x_m = (v / 100.0) if v is not None else None

            # ---- Y ----
            y_m = None
            if i_y_m >= 0 and len(r) > i_y_m:
                y_m = _maybe_float(r[i_y_m])
            elif i_y_cm >= 0 and len(r) > i_y_cm:
                v = _maybe_float(r[i_y_cm])
                y_m = (v / 100.0) if v is not None else None

            # ---- Yaw ----
            yaw_rad = None
            if i_yaw_rad >= 0 and len(r) > i_yaw_rad:
                yaw_rad = _maybe_float(r[i_yaw_rad])
            elif i_yaw_deg >= 0 and len(r) > i_yaw_deg:
                v = _maybe_float(r[i_yaw_deg])
                yaw_rad = math.radians(v) if v is not None else None
            elif i_yaw_generic >= 0 and len(r) > i_yaw_generic:
                yaw_rad = _maybe_float(r[i_yaw_generic])  # assume radians

            # Need at least X and Y to place the chassis
            if x_m is None or y_m is None:
                # Still allow wheel-only keying if you insist, but typical tracks need pose too.
                # continue
                pass

            # Key chassis X/Y
            if x_m is not None:
                obj.location.x = x_m
                obj.keyframe_insert(data_path="location", frame=f, index=0)
            if y_m is not None:
                obj.location.y = y_m
                obj.keyframe_insert(data_path="location", frame=f, index=1)

            # Key yaw (Z euler) if present
            if yaw_rad is not None:
                e = obj.rotation_euler
                e.z = yaw_rad
                obj.rotation_euler = e
                obj.keyframe_insert(data_path="rotation_euler", frame=f, index=2)

            # ---- Wheel rotations ----
            # Left
            thetaL = None
            if i_thetaL_rad >= 0 and len(r) > i_thetaL_rad:
                thetaL = _maybe_float(r[i_thetaL_rad])
            elif i_thetaL_deg >= 0 and len(r) > i_thetaL_deg:
                v = _maybe_float(r[i_thetaL_deg])
                thetaL = math.radians(v) if v is not None else None

            # Right
            thetaR = None
            if i_thetaR_rad >= 0 and len(r) > i_thetaR_rad:
                thetaR = _maybe_float(r[i_thetaR_rad])
            elif i_thetaR_deg >= 0 and len(r) > i_thetaR_deg:
                v = _maybe_float(r[i_thetaR_deg])
                thetaR = math.radians(v) if v is not None else None

            # Apply and key wheel rotations (absolute angles + base pose)
            if wheelL and thetaL is not None:
                _apply_wheel_angle(wheelL, spin_axis, baseL, thetaL)
                wheelL.keyframe_insert(data_path="rotation_euler", frame=f, index=spin_axis)
            if wheelR and thetaR is not None:
                _apply_wheel_angle(wheelR, spin_axis, baseR, thetaR)
                wheelR.keyframe_insert(data_path="rotation_euler", frame=f, index=spin_axis)

            any_keys = True

        if not any_keys:
            self.report({'WARNING'}, "Parsed CSV but no valid rows produced keyframes.")
            return {'CANCELLED'}

        self.report({'INFO'}, f"Imported animation to '{obj.name}' (chassis + wheels).")
        return {'FINISHED'}


# ------------- register -------------

CLASSES = (SG_OT_ImportCSVAnim,)

def register():
    from bpy.utils import register_class
    for c in CLASSES:
        register_class(c)

def unregister():
    from bpy.utils import unregister_class
    for c in reversed(CLASSES):
        unregister_class(c)
