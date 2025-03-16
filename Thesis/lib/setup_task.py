from lib.setup_import_standart import *
from lib.setup_camera import add_camera_overhead
from lib.setup_robot import attach_robot_to_table, set_robot_attach_table

# # Load asset root path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    sys.exit()

class Table:
    """
    Class for handling a table USD asset.
    """
    def __init__(self):
        # Define paths for the table asset.
        self.usd_path = assets_root_path + "/Isaac/Environments/Outdoor/Rivermark/dsready_content/nv_content/common_assets/props_general/table01/table01.usd"
        self.prim_path = "/World/Table"
        self.prim = None

    def add_to_stage(self):
        """
        Add the table USD asset to the stage, verify it, and apply collision.
        """
        print("Adding table USD asset...")
        # Add USD to the stage
        add_reference_to_stage(usd_path=self.usd_path, prim_path=self.prim_path)
        
        # Get the current stage
        stage = get_current_stage()
        print(f"Stage object: {stage}")
        if not isinstance(stage, Usd.Stage):
            raise RuntimeError("Error: Failed to get a valid stage object.")
        
        # Get the prim at the specified path
        sdf_path = Sdf.Path(self.prim_path)  # Convert prim_path to Sdf.Path
        print(f"Retrieving prim at: {sdf_path}")
        self.prim = stage.GetPrimAtPath(sdf_path)
        print(f"Prim object: {self.prim}")
        
        # Validate the prim
        if not self.prim.IsValid():
            print(f"Error: Prim at {self.prim_path} is not valid.")
            return None
        
        print(f"Success: Table prim added at {self.prim_path}")
        apply_collision(self.prim)
        print(f"Success: Table prim added and collision applied at {sdf_path}")
        return self.prim

def add_qr_code_to_table():
    """
    Add a QR code asset to a corner of the table (a Usd.Prim),
    then position it at a desired offset.
    """
    # Create a unique prim path for the QR code as a child of the table
    # qr_code_path = table_prim.GetPath().AppendChild("QR_board")
    prim_path = f"/World/QR_board" 
    qr_usd_path = "E:/NVIDIA/isaacsim/myscripts/Thesis/lib/object_detect_lib/resources/checkerboard_2.usdz"

    print("Adding QR code USD asset to the table...")
    add_reference_to_stage(usd_path=qr_usd_path, prim_path=prim_path)
    
    qr_prim = get_prim_at_path(prim_path)
    print("qr_prim: ", qr_prim)
    if not qr_prim.IsValid():
        print(f"Error: QR Code prim at {prim_path} is invalid.")
        return

    # Position the QR code (tweak offsets as needed)
    position = (0.34, 0.35, 0.637)  # Offset from the table corner
    UsdGeom.Xformable(qr_prim).AddTranslateOp().Set(position)
    print(f"QR Code added at {prim_path} with a corner offset.")

    # Wrap the prim for transformation operations
    xformable = UsdGeom.Xformable(qr_prim)

    # Scale down the object (e.g., to half size in all dimensions)
    scale_op = xformable.AddScaleOp()
    scale_op.Set(Gf.Vec3f(0.0024, 0.0024, 0.0024))

    # Rotate the object 90 degrees about the x-axis.
    # The API uses degrees, so you can simply set 90.
    rotate_x_op = xformable.AddRotateXOp()
    rotate_x_op.Set(90)
    print(f"Applied scale and 90° x-axis rotation to prim: {prim_path}")

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
        # usd_path= assets_root_path + "/Isaac/Props/Blocks/nvidia_cube.usd",
        usd_path= assets_root_path + "/Isaac/Props/Blocks/MultiColorCube/multi_color_cube_instanceable.usd", # multicolor cube
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
    cube_positions = (0.0, 0.0, 0.9)
    # cube_positions = [(0.0, 0.0, 0.825), (0.1, 0.1, 0.825), (-0.1, 0.1, 0.825)]
    # for i, pos in enumerate(cube_positions):
    #     add_cube(i, pos)  # Pass unique index and position
    add_cube(1, cube_positions)  # Pass unique index and position


def set_the_scene():   
    # Setup the scene by adding ground plane, table, QR code, cubes, and configuring the robot and camera.
    world.scene.add_default_ground_plane()  # add ground plane
    
    # Use the Table class to add the table to the stage
    table_obj = Table()
    table_prim = table_obj.add_to_stage()
    
    add_qr_code_to_table()  
    add_cubes()
    set_robot_attach_table()
    camera_overhead = add_camera_overhead()