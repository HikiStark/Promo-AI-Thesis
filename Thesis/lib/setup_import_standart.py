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

def apply_collision(prim):
    """
    Apply collision and physics properties to a given prim.

    Args:
        prim (Usd.Prim): The USD prim to which collision will be applied.
    """
    # Apply the collision API
    collision_api = UsdPhysics.CollisionAPI.Apply(prim)
    print("Collision API applied.")

    # Set physics attributes
    collision_type_attr = collision_api.GetCollisionEnabledAttr()
    if not collision_type_attr:
        collision_api.CreateCollisionEnabledAttr(True)
        print("Collision enabled attribute created.")

    # Add physics mass (optional for dynamic bodies)
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
        print("Rigid body API applied to enable physics.")

    print("Collision successfully applied to prim.")


def calc_object_midpoint(prim):
    stage = get_current_stage()
    object_prim = get_prim_at_path(prim)

    if not object_prim.IsValid():
        raise ValueError(f"Error: Table prim at {object_prim} is not valid.")
     
    # Get the table's bounding box to calculate the top surface position
    object_geom = UsdGeom.Boundable(object_prim)
    bbox = object_geom.ComputeWorldBound(0, "default")
    object_min = bbox.GetRange().GetMin()
    object_max = bbox.GetRange().GetMax()

    object_top_mid_position = Gf.Vec3d(
        (object_min[0] + object_max[0]) / 2,  # Center along X
        (object_min[1] + object_max[1]) / 2,  # Center along Y
        object_max[2],  # Top surface Z
    )

    object_mid_X = object_top_mid_position[0]
    object_mid_Y = object_top_mid_position[1]
    object_mid_Z = object_top_mid_position[2]

    return object_top_mid_position, object_mid_X, object_mid_Y, object_mid_Z