import sys
import carb
import argparse
import numpy as np
from omni.isaac.nucleus import get_assets_root_path

from pxr import UsdPhysics, PhysxSchema, Usd, Sdf, UsdGeom, Gf 

from omni.isaac.core import World
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.physics_context import PhysicsContext
from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.objects import VisualCuboid, DynamicCuboid, FixedCuboid
# import omni.isaac.core.controllers.articulation_controller.ArticulationController

from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import SurfaceGripper

from omni.isaac.sensor import Camera

from omni.isaac.universal_robots import UR10
from omni.isaac.universal_robots.tasks import FollowTarget
from omni.isaac.universal_robots.controllers import StackingController
from omni.isaac.universal_robots.controllers.rmpflow_controller import RMPFlowController
from omni.isaac.universal_robots.controllers.pick_place_controller import (
    PickPlaceController,
)

world = World(stage_units_in_meters=1.0)