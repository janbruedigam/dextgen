import random

from gym import utils
import numpy as np

from envs.parallel_jaw.flat_base import FlatPJBase
from envs.rotations import axisangle2quat, quatmultiply


class FlatPJAll(FlatPJBase, utils.EzPickle):

    def __init__(self, object_size_range: float = 0):
        FlatPJBase.__init__(self, object_name="cube", object_size_range=object_size_range)
        utils.EzPickle.__init__(self, object_size_range=object_size_range)

    def _env_setup(self, initial_qpos: np.ndarray):
        self.object_name = random.choice(("cube", "cylinder", "sphere", "mesh"))
        super()._env_setup(initial_qpos)

    def _sample_object_pose(self) -> np.ndarray:
        object_pose = self.sim.data.get_joint_qpos(self.object_name + ":joint")
        object_pose[:2] = self.sim.data.get_body_xpos("table0")[:2]
        object_pose[2] = self.height_offset
        if self.object_name == "cube":
            object_rot = self.np_random.rand(4)
            object_rot[1:3] = 0
            object_rot /= np.linalg.norm(object_rot)
        elif self.object_name == "sphere":
            object_rot = np.array([1, 0, 0, 0])
        elif self.object_name in ("cylinder", "mesh"):
            if self.np_random.rand() < 0.5:
                rot_y = axisangle2quat(0, 1, 0, np.pi / 2)
                rot_z = axisangle2quat(0, 0, 1, self.np_random.rand() * np.pi)
                object_rot = quatmultiply(rot_z, rot_y)
            else:
                object_rot = axisangle2quat(0, 0, 1, self.np_random.rand() * np.pi)
        else:
            raise RuntimeError(f"Object {self.object_name} not supported")
        object_pose[3:7] = object_rot
        return object_pose