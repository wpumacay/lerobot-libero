from .mounted_panda import MountedPanda
from .on_the_ground_panda import OnTheGroundPanda
from .franka_droid import OnTheGroundFrankaDroid, FrankaDroidGripper

from robosuite.robots.single_arm import SingleArm
from robosuite.robots import ROBOT_CLASS_MAPPING
from robosuite.models.grippers import ALL_GRIPPERS, GRIPPER_MAPPING

ROBOT_CLASS_MAPPING.update(
    {
        "MountedPanda": SingleArm,
        "OnTheGroundPanda": SingleArm,
    }
)

GRIPPER_MAPPING.update({"FrankaDroidGripper": FrankaDroidGripper})
