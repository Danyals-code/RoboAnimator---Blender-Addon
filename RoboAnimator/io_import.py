# io_import.py — Import animation (x, y, yaw) from CSV and attach to object
# Accepts CSV headers like: t,x_m,y_m,yaw_deg,... OR t,x_cm,y_cm,yaw_rad,...
# Keys object location (X,Y) and rotation_euler.z (yaw). Attaches CSV as a Text datablock.

import bpy
import csv
import math
from bpy_extras.io_utils import ImportHelper


# ---------- helpers ----------
def _find_col(header, *cands):
    """Return column index for first match in cands. Case-insensitive. -1 if not found."""
    hl = [h.strip().lower() for h in header]
    for cand in cands:
        if isinstance(cand, (list, tuple)):
            for c in cand:
                if c in hl:
                    return hl.index(c)
        else:
            if cand in hl:
                return hl.index(cand)
    return -1


def _units_from_header(header):
    """Infer length scale and yaw unit from header names."""
    hl = [h.strip().lower() for h in header]
    length_scale = 1.0   # meters by default; if cm, we scale by 0.01
    angle_is_radians = True
    for h in hl:
        if h.startswith("x_cm") or h.startswith("y_cm"):
            length_scale = 0.01
        if h.startswith("yaw_deg"):
            angle_is_radians = False
        if h.startswith("yaw_rad"):
            angle_is_radians = True
    return length_scale, angle_is_radians


def _iter_rows(path):
    """Read CSV, skipping lines starting with '#'. Return (header, rows)."""
    with open(path, 'r', encoding='utf-8') as f:
        lines = [ln for ln in f.readlines() if not ln.lstrip().startswith('#')]
    reader = csv.reader(lines)
    header = next(reader, None)
    if header is None:
        raise ValueError("CSV has no header row.")
    rows = [r for r in reader if r]
    return header, rows


def _to_frame(t_seconds, fps, start_frame):
    return int(round(start_frame + float(t_seconds) * fps))


def _ensure_text(name, content):
    """Create/replace a Text datablock with CSV content and return it."""
    txt = bpy.data.texts.get(name)
    if txt is None:
        txt = bpy.data.texts.new(name)
    txt.clear()
    txt.write(content)
    return txt


# ---------- operator ----------
class SG_OT_ImportCSVAnim(bpy.types.Operator, ImportHelper):
    """Import (x,y,yaw) animation from CSV and keyframe the active object"""
    bl_idname = "segway.import_csv_anim"
    bl_label = "Import CSV Animation"
    bl_description = "Attach CSV to the selected object and keyframe location (x,y) and yaw (Z-rotation)"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".csv"
    filter_glob: bpy.props.StringProperty(default="*.csv", options={'HIDDEN'})

    apply_linear_fcurves: bpy.props.BoolProperty(
        name="Linear Interpolation",
        description="Set imported F-Curves to LINEAR interpolation",
        default=True,
    )
    fps: bpy.props.IntProperty(
        name="FPS (if using t column)",
        default=24, min=1, max=480
    )
    start_frame: bpy.props.IntProperty(
        name="Start Frame",
        default=1, min=-100000, max=100000
    )

    def execute(self, context):
        obj = context.object
        if obj is None:
            self.report({'ERROR'}, "No active object selected.")
            return {'CANCELLED'}

        # Read CSV
        try:
            header, rows = _iter_rows(self.filepath)
        except Exception as e:
            self.report({'ERROR'}, f"Failed to read CSV: {e}")
            return {'CANCELLED'}

        # Find columns
        ci_t     = _find_col(header, "t", "time", "seconds")
        ci_frame = _find_col(header, "frame", "f")
        ci_x     = _find_col(header, "x_m", "x_cm", "x")
        ci_y     = _find_col(header, "y_m", "y_cm", "y")
        ci_yaw_d = _find_col(header, "yaw_deg")
        ci_yaw_r = _find_col(header, "yaw_rad", "yaw")

        if ci_x < 0 or ci_y < 0 or (ci_yaw_d < 0 and ci_yaw_r < 0):
            self.report({'ERROR'}, "CSV must include columns for x, y, and yaw (deg or rad).")
            return {'CANCELLED'}

        length_scale, angle_is_radians = _units_from_header(header)
        if ci_yaw_d >= 0:  # explicit deg column wins
            angle_is_radians = False

        # Attach original CSV as Text for provenance
        try:
            with open(self.filepath, 'r', encoding='utf-8') as f:
                raw = f.read()
        except Exception as e:
            self.report({'ERROR'}, f"Failed to read CSV content: {e}")
            return {'CANCELLED'}

        txt_name = f"{obj.name}_anim.csv"
        txt = _ensure_text(txt_name, raw)
        obj['roboanim_csv_text'] = txt.name  # store reference on the object

        # Ensure action
        if obj.animation_data is None:
            obj.animation_data_create()
        if obj.animation_data.action is None:
            obj.animation_data.action = bpy.data.actions.new(f"{obj.name}_ImportedCSV")

        # Clear existing curves on the paths we touch (location.*, rotation_euler.z)
        action = obj.animation_data.action
        to_remove = [fc for fc in action.fcurves if fc.data_path in ("location", "rotation_euler")]
        for fc in to_remove:
            action.fcurves.remove(fc)

        # Insert keyframes
        frame_counter = self.start_frame
        imported_count = 0
        for r in rows:
            try:
                x = float(r[ci_x]) * length_scale
                y = float(r[ci_y]) * length_scale
                if ci_yaw_r >= 0:
                    yaw = float(r[ci_yaw_r])  # radians
                else:
                    yaw = math.radians(float(r[ci_yaw_d]))

                if ci_frame >= 0:
                    f = int(float(r[ci_frame]))
                elif ci_t >= 0:
                    f = _to_frame(r[ci_t], self.fps, self.start_frame)
                else:
                    f = frame_counter
                    frame_counter += 1
            except Exception as e:
                self.report({'WARNING'}, f"Skipping row due to parse error: {e}")
                continue

            obj.location.x = x
            obj.location.y = y
            # keep Z location untouched
            eul = obj.rotation_euler
            eul.z = yaw
            obj.rotation_euler = eul

            obj.keyframe_insert(data_path="location", frame=f)
            obj.keyframe_insert(data_path="rotation_euler", frame=f, index=2)  # z only
            imported_count += 1

        # Set interpolation
        if self.apply_linear_fcurves and obj.animation_data and obj.animation_data.action:
            for fc in obj.animation_data.action.fcurves:
                for kp in fc.keyframe_points:
                    kp.interpolation = 'LINEAR'

        self.report({'INFO'}, f"Imported {imported_count} rows into {obj.name}. CSV attached as Text '{txt.name}'.")
        return {'FINISHED'}


# ---------- register ----------
CLASSES = (SG_OT_ImportCSVAnim,)

def register():
    from bpy.utils import register_class
    for c in CLASSES:
        register_class(c)

def unregister():
    from bpy.utils import unregister_class
    for c in reversed(CLASSES):
        unregister_class(c)
