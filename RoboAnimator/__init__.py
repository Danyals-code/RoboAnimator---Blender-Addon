bl_info = {
    "name": "True RoboAnimator",
    "author": "Danyal S.",
    "version": (0, 4, 0),
    "blender": (4, 2, 0),
    "location": "3D Viewport > N-Panel > True RoboAnimator",
    "description": "Six-file core with feasibility panel.",
    "category": "Animation",
}

import bpy

from .selection import Props, register_props, unregister_props
from .feasibility import (
    FeasProps, register_feas_props, unregister_feas_props,
)
from .operators import (
    RefreshWheels,
    AutoRadius,
    ValidatePath,
    BuildCache,
    AttachDrivers,
    ValidateMotion,
    AutoCorrectPath,
    RevertAutoCorrect,
)
from .ui import MainPanel

_CLASSES = (
    # props
    Props,
    FeasProps,
    # ops
    RefreshWheels,
    AutoRadius,
    ValidatePath,
    BuildCache,
    AttachDrivers,
    ValidateMotion,
    AutoCorrectPath,
    RevertAutoCorrect,
    # ui
    MainPanel,
)

def register():
    for c in _CLASSES:
        bpy.utils.register_class(c)
    register_props()
    register_feas_props()

def unregister():
    unregister_feas_props()
    unregister_props()
    for c in reversed(_CLASSES):
        bpy.utils.unregister_class(c)

if __name__ == "__main__":
    register()
