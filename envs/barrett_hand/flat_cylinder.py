"""FlatBarrettCylinder environment module."""
from pathlib import Path
from typing import Optional

from gym import utils
import numpy as np

from envs.barrett_hand.flat_base import FlatBarrettBase
from envs.rotations import axisangle2quat, quat_mul

MODEL_XML_PATH = str(Path("BarrettHand", "flat_cylinder.xml"))


class FlatBarrettCylinder(FlatBarrettBase, utils.EzPickle):
    """FlatBarrettCylinder environment class."""

    def __init__(self,
                 n_eigengrasps: Optional[int] = None,
                 p_rot: float = 0.5,
                 object_size_multiplier: float = 1.,
                 object_size_range: float = 0.):
        """Initialize a BarrettHand cylinder environment.

        Args:
            n_eigengrasps: Number of eigengrasps to use.
            p_rot: Ratio of laying cylinders in the episodes.
            object_size_multiplier: Optional multiplier to change object sizes by a fixed amount.
            object_size_range: Optional range to randomly enlarge/shrink object sizes.
        """
        FlatBarrettBase.__init__(self,
                                 object_name="cylinder",
                                 model_xml_path=MODEL_XML_PATH,
                                 n_eigengrasps=n_eigengrasps,
                                 object_size_multiplier=object_size_multiplier,
                                 object_size_range=object_size_range)
        self.p_rot = p_rot
        utils.EzPickle.__init__(self,
                                n_eigengrasps=n_eigengrasps,
                                p_rot=p_rot,
                                object_size_multiplier=object_size_multiplier,
                                object_size_range=object_size_range)

    def _sample_object_pose(self) -> np.ndarray:
        object_pose = super()._sample_object_pose()
        # Random rotation around z axis
        if self.np_random.rand() < self.p_rot:
            y_rot = axisangle2quat(0, 1, 0, np.pi / 2)
            z_rot = axisangle2quat(0, 0, 1, self.np_random.rand() * np.pi)
            object_rot = quat_mul(z_rot, y_rot)
        else:
            object_rot = axisangle2quat(0, 0, 1, self.np_random.rand() * np.pi)
        object_pose[3:7] = object_rot
        return object_pose
