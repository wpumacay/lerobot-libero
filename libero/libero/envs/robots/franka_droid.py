from collections import OrderedDict
from pathlib import Path

import numpy as np

from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion

LIBERO_ASSETS_DIR = Path(__file__).parent.parent.parent
FRANKA_DROID_DIR = LIBERO_ASSETS_DIR / "assets" / "robots" / "franka_droid"

FRANKA_DROID_ROBOT_XML = FRANKA_DROID_DIR / "model_no_gripper.xml"
FRANKA_DROID_GRIPPER_XML = FRANKA_DROID_DIR / "robotiq_2f85_v4" / "2f85.xml"

class OnTheGroundFrankaDroid(ManipulatorModel):
    """
    Panda is a sensitive single-arm robot designed by Franka.
    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, idn=0):
        super().__init__(FRANKA_DROID_ROBOT_XML.resolve().as_posix(), idn=idn)

    @property
    def default_mount(self):
        return None

    @property
    def default_gripper(self):
        return "FrankaDroidGripper"

    @property
    def default_controller_config(self):
        return "default_panda"

    @property
    def init_qpos(self):
        return np.array([0, -1.61037389e-01, 0.00, -2.44459747e00, 0.00, 2.22675220e00, np.pi / 4])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
            "coffee_table": lambda table_length: (-0.16 - table_length / 2, 0, 0.41),
            "living_room_table": lambda table_length: (
                -0.16 - table_length / 2,
                0,
                0.42,
            ),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"

    @property
    def _eef_name(self):  # type: ignore
        return "eef_link"


class FrankaDroidGripper(GripperModel):
    def __init__(self, idn=0):
        super().__init__(FRANKA_DROID_GRIPPER_XML.resolve().as_posix(), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    @property
    def _important_geoms(self):
        return {}
