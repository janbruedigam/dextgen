"""FlatPJSphere environment module."""
from pathlib import Path

from gym import utils

from envs.parallel_jaw.flat_base import FlatPJBase

import envs.init_qpos

MODEL_XML_PATH = str(Path("PJ", "flat_sphere.xml"))


class FlatPJSphere(FlatPJBase, utils.EzPickle):
    """FlatPJSphere environment class."""

    def __init__(self, init_random: bool = True,  
                 object_size_multiplier: float = 1., 
                 object_size_range: float = 0., 
                 initial_qpos = envs.init_qpos.DEFAULT_INITIAL_QPOS_PJ):
        """Initialize a parallel jaw sphere environment.

        Args:
            object_size_multiplier: Optional multiplier to change object sizes by a fixed amount.
            object_size_range: Optional range to randomly enlarge/shrink object sizes.
        """
        FlatPJBase.__init__(self,
                            object_name="sphere",
                            init_random=init_random,
                            initial_qpos=initial_qpos,
                            model_xml_path=MODEL_XML_PATH,
                            object_size_multiplier=object_size_multiplier,
                            object_size_range=object_size_range)
        utils.EzPickle.__init__(self,
                                object_size_multiplier=object_size_multiplier,
                                object_size_range=object_size_range)
