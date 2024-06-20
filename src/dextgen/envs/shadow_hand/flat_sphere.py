"""FlatSHSphere environment module."""
from pathlib import Path
from typing import Optional

from gym import utils

from dextgen.envs.shadow_hand.flat_base import FlatSHBase

import dextgen.envs.init_qpos

MODEL_XML_PATH = str(Path("ShadowHand", "flat_sphere.xml"))


class FlatSHSphere(FlatSHBase, utils.EzPickle):
    """FlatSHSphere environment class."""

    def __init__(self, init_random: bool = True,
                 n_eigengrasps: Optional[int] = None,
                 object_size_multiplier: float = 1.,
                 object_size_range: float = 0., 
                 initial_qpos = dextgen.envs.init_qpos.DEFAULT_INITIAL_QPOS_SH):
        """Initialize a ShadowHand sphere environment.

        Args:
            n_eigengrasps: Number of eigengrasps to use.
            object_size_multiplier: Optional multiplier to change object sizes by a fixed amount.
            object_size_range: Optional range to randomly enlarge/shrink object sizes.
        """
        FlatSHBase.__init__(self,
                            object_name="sphere",
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
