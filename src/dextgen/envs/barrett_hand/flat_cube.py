"""FlatBarrettCube environment module."""
from pathlib import Path
from typing import Optional

from gym import utils
import numpy as np

from dextgen.envs.barrett_hand.flat_base import FlatBarrettBase

import dextgen.envs.init_qpos

MODEL_XML_PATH = str(Path("BarrettHand", "flat_cube.xml"))


class FlatBarrettCube(FlatBarrettBase, utils.EzPickle):
    """FlatBarrettCube environment class."""

    def __init__(self, init_random: bool = True,
                 n_eigengrasps: Optional[int] = None,
                 object_size_multiplier: float = 1.,
                 object_size_range: float = 0., 
                 initial_qpos = dextgen.envs.init_qpos.DEFAULT_INITIAL_QPOS_Barrett):
        """Initialize a BarrettHand cube environment.

        Args:
            n_eigengrasps: Number of eigengrasps to use.
            object_size_multiplier: Optional multiplier to change object sizes by a fixed amount.
            object_size_range: Optional range to randomly enlarge/shrink object sizes.
        """
        FlatBarrettBase.__init__(self,
                                 object_name="cube",
                                 init_random=init_random,
                                 initial_qpos=initial_qpos,
                                 model_xml_path=MODEL_XML_PATH,
                                 n_eigengrasps=n_eigengrasps,
                                 object_size_multiplier=object_size_multiplier,
                                 object_size_range=object_size_range)
        utils.EzPickle.__init__(self,
                                n_eigengrasps=n_eigengrasps,
                                object_size_multiplier=object_size_multiplier,
                                object_size_range=object_size_range)

    def _sample_object_pose(self) -> np.ndarray:
        object_pose = super()._sample_object_pose()
        # Random rotation around z axis
        object_pose[3] = self.np_random.rand()
        object_pose[6] = self.np_random.rand()
        object_pose[3:7] /= np.linalg.norm(object_pose[3:7])
        return object_pose
