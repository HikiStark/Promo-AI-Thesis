from lib.setup_import_standart import *
from lib.setup_camera import add_camera_overhead
from lib.setup_robot import attach_robot_to_table, set_robot_attach_table

# # Load asset root path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    sys.exit()

def add_table():
    """
    Add a table USD asset to the stage and verify it.
    """
    # Define paths
    usd_path = assets_root_path + "/Isaac/Environments/Outdoor/Rivermark/dsready_content/nv_content/common_assets/props_general/table01/table01.usd"
    prim_path = "/World/Table"

    print("Adding table USD asset...")

    # Add USD to the stage
    add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

    # Get the current stage
    stage = get_current_stage()
    print(f"Stage object: {stage}")

    # Debug: Check if the stage object is valid
    if not isinstance(stage, Usd.Stage):
        raise RuntimeError("Error: Failed to get a valid stage object.")

    # Get the prim at the specified path
    sdf_path = Sdf.Path(prim_path)  # Convert prim_path to Sdf.Path
    print(f"Retrieving prim at: {sdf_path}")

    prim = stage.GetPrimAtPath(sdf_path)
    print(f"Prim object: {prim}")

    # Validate the prim
    if not prim.IsValid():
        print(f"Error: Prim at {prim_path} is not valid.")
        return

    print(f"Success: Table prim added at {prim_path}")

    apply_collision(prim)
    print(f"Success: Table prim added and collision applied at {sdf_path}")

    return prim

# TODO: Change the table to an object class
# TODO: Move add_table function under the class
# TODO: Change all the occurances of the function so it works accordingly

table_prim = "/World/Table"

def add_qr_code_to_table(table_prim):
    """
    Add a QR code asset to a corner of the table.
    This function creates a new prim under the table prim,
    loads the QR code asset, and positions it at a desired offset.
    """
    # Create a unique prim path for the QR code as a child of the table
    qr_code_path = table_prim.GetPath().AppendChild("QR_Code")
    qr_usd_path = assets_root_path + "/Isaac/Props/QR_Code/qr_code.usd"  # Ensure this asset exists

    print("Adding QR code USD asset to the table...")

    # Add the QR code to the stage as a child of the table
    add_reference_to_stage(usd_path=qr_usd_path, prim_path=str(qr_code_path))
    
    # Retrieve the QR code prim from the stage
    qr_prim = get_prim_at_path(str(qr_code_path))
    if not qr_prim.IsValid():
        print(f"Error: QR Code prim at {qr_code_path} is invalid.")
        return

    # Set the translation to position the QR code at the table's corner.
    # These offset values (e.g., (-0.3, 0.3, 0.05)) might need adjustment based on your table's dimensions.
    UsdGeom.Xformable(qr_prim).AddTranslateOp().Set((-0.3, 0.3, 0.05))
    print(f"QR Code added at {qr_code_path} with a corner offset.")

    return qr_prim


def add_cube(index, position):
    """
    Add a cube with a unique name at the specified position.

    Args:
        index (int): Unique index for naming the cube.
        position (tuple): (x, y, z) position to place the cube.
    """
    prim_path = f"/World/cube_{index}"  # Unique path for each cube
    add_reference_to_stage(
        usd_path= assets_root_path + "/Isaac/Props/Blocks/nvidia_cube.usd",
        # usd_path= assets_root_path + "/Isaac/Props/Blocks/MultiColorCube/multi_color_cube_instanceable.usd", # multicolor cube
        # usd_path= assets_root_path + "/Isaac/Props/Blocks/DexCube/dex_cube_instanceable.usd", # dex cube
        prim_path=prim_path,
    )

    # Move cube to the desired position
    cube_prim = get_prim_at_path(prim_path)
    if not cube_prim.IsValid():
        print(f"Error: Cube at {prim_path} is invalid.")
        return

    UsdGeom.Xformable(cube_prim).AddTranslateOp().Set(position)
    print(f"Cube {index} added at position {position}")


def add_cubes():
    """
    Add multiple cubes at different positions.
    """
    cube_positions = [(0.0, 0.0, 0.825), (0.1, 0.1, 0.825), (-0.1, 0.1, 0.825)]
    for i, pos in enumerate(cube_positions):
        add_cube(i, pos)  # Pass unique index and position


def set_the_scene():
    world.scene.add_default_ground_plane()  # add ground plane
    table = add_table()
    add_cubes()
    set_robot_attach_table()
    camera_overhead = add_camera_overhead()