from lib.setup_import_standart import *
from lib.setup_camera import add_camera_overhead
from lib.setup_robot import attach_robot_to_table, set_robot_attach_table

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load asset root path
assets_root_path: Optional[str] = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    sys.exit()

# Set up ZMQ publisher
context = zmq.Context()


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


class QRCode:
    """
    Class for handling a QR code USD asset to be attached to the table.
    """

    def __init__(self) -> None:
        self.prim_path: str = "/World/QR_board"
        self.qr_usd_path: str = "E:/NVIDIA/isaacsim/myscripts/Thesis/lib/object_detect_lib/resources/checkerboard_2.usdz"
        self.prim: Optional[Usd.Prim] = None

    def add_to_stage(self) -> Optional[Usd.Prim]:
        """
        Add the QR code USD asset to the stage and apply transformation.
        """
        logger.info("Adding QR code USD asset to the table...")
        add_reference_to_stage(usd_path=self.qr_usd_path, prim_path=self.prim_path)

        self.prim = get_prim_at_path(self.prim_path)
        logger.info(f"QR prim: {self.prim}")
        if not self.prim.IsValid():
            logger.error(f"Error: QR Code prim at {self.prim_path} is invalid.")
            return None

        # Position the QR code
        position: Tuple[float, float, float] = (0.34, 0.35, 0.637)
        UsdGeom.Xformable(self.prim).AddTranslateOp().Set(position)
        logger.info(f"QR Code added at {self.prim_path} with offset {position}.")

        # Apply scale and rotation
        xformable = UsdGeom.Xformable(self.prim)
        scale_op = xformable.AddScaleOp()
        scale_op.Set(Gf.Vec3f(0.0024, 0.0024, 0.0024))
        rotate_x_op = xformable.AddRotateXOp()
        rotate_x_op.Set(90)
        logger.info(f"Applied scale and 90° x-axis rotation to QR prim: {self.prim_path}")
        return self.prim


class Cube:
    """
    Class for handling a cube USD asset.
    """

    def __init__(self, index: int, position: Tuple[float, float, float]) -> None:
        self.index: int = index
        self.position: Tuple[float, float, float] = position
        self.prim_path: str = f"/World/cube_{index}"
        self.usd_path: str = (
            assets_root_path
            + "/Isaac/Props/Blocks/MultiColorCube/multi_color_cube_instanceable.usd"  # multicolor cube
            # assets_root_path + "/Isaac/Props/Blocks/nvidia_cube.usd" # nvidia cube
            # assets_root_path + "/Isaac/Props/Blocks/DexCube/dex_cube_instanceable.usd" # dex cube
        )
        self.prim: Optional[Usd.Prim] = None

    def add_to_stage(self) -> Optional[Usd.Prim]:
        """
        Add the cube USD asset to the stage and position it.
        """
        add_reference_to_stage(usd_path=self.usd_path, prim_path=self.prim_path)
        self.prim = get_prim_at_path(self.prim_path)
        if not self.prim.IsValid():
            logger.error(f"Error: Cube at {self.prim_path} is invalid.")
            return None

        UsdGeom.Xformable(self.prim).AddTranslateOp().Set(self.position)
        logger.info(f"Cube {self.index} added at position {self.position}")
        return self.prim


class Scene:
    """
    Class to manage the entire scene setup.
    """

    def __init__(self) -> None:
        self.table = Table()
        self.qr_code = QRCode()
        self.cubes: List[Cube] = []
        self.camera = None

    def add_ground_plane(self) -> None:
        world.scene.add_default_ground_plane()
        logger.info("Default ground plane added.")

    def add_table(self) -> Optional[Usd.Prim]:
        return self.table.add_to_stage()

    def add_qr_code(self) -> Optional[Usd.Prim]:
        return self.qr_code.add_to_stage()

    def add_cube(self, index: int, position: Tuple[float, float, float]) -> Optional[Usd.Prim]:
        cube = Cube(index=index, position=position)
        self.cubes.append(cube)
        return cube.add_to_stage()

    def add_cubes(self) -> None:
        # Example: Adding a single cube; extend this for multiple cubes as needed.
        self.add_cube(1, (0.0, 0.0, 0.9))
        # cube_positions = [(0.0, 0.0, 0.9), (0.1, 0.1, 0.9), (-0.1, 0.1, 0.9)]
        # for i, pos in enumerate(cube_positions):
        #     self.add_cube(i, pos)  # Pass unique index and position

    def setup_robot(self) -> None:
        set_robot_attach_table()
        logger.info("Robot and camera setup completed.")

    def add_camera(self, sim_app) -> None:
        self.camera = add_camera_overhead(sim_app)
        logger.info("Overhead camera added")

    def setup(self, sim_app) -> None:
        self.add_ground_plane()
        self.add_table()
        self.add_qr_code()
        self.add_cubes()
        self.setup_robot()
        self.add_camera(sim_app)
        logger.info("Scene setup completed.")


def initialize_publisher():
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 1)  # Limit backlog
    socket.bind("tcp://*:5555")
    logger.info("Publisher socket initialized on port 5555")
    return socket


def initialize_command_receiver():
    cmd_socket = context.socket(zmq.PULL)
    cmd_socket.bind("tcp://*:5560")  # Bind on a new port (5560)
    logger.info("Command receiver socket initialized on port 5560")
    return cmd_socket


def set_the_scene(sim_app):
    # Main function to initialize and setup the scene.
    print("Setting up the scene..." + str(sim_app))
    scene = Scene()
    scene.setup(sim_app)

    return scene


if __name__ == "__main__":

    set_the_scene()
