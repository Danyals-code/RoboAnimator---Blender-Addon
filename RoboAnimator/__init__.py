bl_info = {
    "name": "True RoboAnimator",
    "author": "Danyal S.",
    "version": (0, 3, 4),
    "blender": (4, 2, 0),
    "location": "3D Viewport > N-Panel > True RoboAnimator",
    "description": "Minimal six-file core.",
    "category": "Animation",
}

import bpy

from .selection import Props, register_props, unregister_props
from .operators import (
    RefreshWheels,
    AutoRadius,
    ValidatePath,
    BuildCache,
    AttachDrivers,
)
from .ui import MainPanel

_CLASSES = (
    Props,
    RefreshWheels,
    AutoRadius,
    ValidatePath,
    BuildCache,
    AttachDrivers,
    MainPanel,
)

def register():
    for c in _CLASSES:
        bpy.utils.register_class(c)
    register_props()

def unregister():
    unregister_props()
    for c in reversed(_CLASSES):
        bpy.utils.unregister_class(c)

if __name__ == "__main__":
    register()
