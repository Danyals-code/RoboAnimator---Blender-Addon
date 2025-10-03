bl_info = {
    "name": "True RoboAnimator",
    "author": "Danyal S.",
    "version": (1, 0, 1),
    "blender": (4, 2, 0),
    "location": "3D Viewport > N-Panel > True RoboAnimator",
    "description": "Engineering-accurate animation toolkit for differential-drive robots.",
    "category": "Animation",
}

import importlib
import sys

# Local module imports (split files)
from . import (
    motion_check,
    kinematics_rpm,
    io_export,
    ops_ui,
    utils,
)

# Keep all registerable classes here
modules = (
    motion_check,
    kinematics_rpm,
    io_export,
    ops_ui,
    utils,
)


def register():
    from bpy.utils import register_class
    import bpy

    # Register property group (SG_Props) first
    if hasattr(motion_check, "SG_Props"):
        bpy.utils.register_class(motion_check.SG_Props)
        bpy.types.Scene.sg_props = bpy.props.PointerProperty(type=motion_check.SG_Props)

    # Register everything else
    for m in modules:
        if hasattr(m, "register"):
            m.register()


def unregister():
    from bpy.utils import unregister_class
    import bpy

    # Unregister in reverse
    for m in reversed(modules):
        if hasattr(m, "unregister"):
            m.unregister()

    if hasattr(motion_check, "SG_Props"):
        del bpy.types.Scene.sg_props
        bpy.utils.unregister_class(motion_check.SG_Props)
