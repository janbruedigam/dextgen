"""Test a previously trained agent on an OpenAI gym environment.

The MuJoCoVideoRecorder is a wrapper around OpenAI's gym VideoRecorder.
"""

import numpy as np
from pathlib import Path
import time
from typing import Optional, Tuple, Any

import pickle
import torch
import gym
from gym.wrappers.monitoring.video_recorder import VideoRecorder
import mujoco_py
import envs  # Import registers environments with gym  # noqa: F401

from mp_rl.core.utils import unwrap_obs
from mp_rl.core.actor import PosePolicyNet, DDP
from parse_args import parse_args


class MujocoVideoRecorder(VideoRecorder):
    """Record videos in Mujoco with adaptive resolution based on the gym VideoRecoder class."""

    def __init__(self, *args: Any, resolution: Optional[Tuple[int]] = None, **kwargs: Any):
        """Initialize a recorder with specific resolution.

        Args:
            args: VideoRecorder arguments.
            resolution: Resolution tuple.
            kwargs: VideoRecorder keyword arguments.
        """
        self.encoder = None
        Path(kwargs["path"]).parent.mkdir(parents=True, exist_ok=True)
        super().__init__(*args, **kwargs)
        self.resolution = resolution

if __name__ == "__main__":
    args = parse_args()

    init_random = args.init_random == "y"
    init_goal = args.init_goal.split(',')
    init_goal = np.array([float(i) for i in init_goal])
    init_object = args.init_object.split(',')
    init_object = np.array([float(i) for i in init_object])
    init_jointangles = args.init_jointangles.split(',')
    init_jointangles = np.array([float(i) for i in init_jointangles])
    init_gripper = float(args.init_gripper)
    print(args.env)

    ros_initial_qpos = {
        "panda_joint1": init_jointangles[0],
        "panda_joint2": init_jointangles[1],
        "panda_joint3": init_jointangles[2],
        "panda_joint4": init_jointangles[3],
        "panda_joint5": init_jointangles[4],
        "panda_joint6": init_jointangles[5],
        "panda_joint7": init_jointangles[6],
        "robot0:r_gripper_finger_joint": init_gripper,
        "robot0:l_gripper_finger_joint": init_gripper,
        "cube:joint": init_object,
        "cylinder:joint": init_object,
        "sphere:joint": init_object,
        "mesh:joint": init_object
    }
    
    if init_random:
        env = gym.make(args.env, **args.kwargs) if hasattr(args, "kwargs") else gym.make(args.env, init_random=init_random, initial_qpos=ros_initial_qpos)
    else:
        env = gym.make(args.env, **args.kwargs) if hasattr(args, "kwargs") else gym.make(args.env)
    size_g = len(env.observation_space["desired_goal"].low)
    size_s = len(env.observation_space["observation"].low) + size_g
    size_a = len(env.action_space.low)
    if args.actor_net_type == "DDP":
        actor = DDP(size_s, size_a, args.actor_net_nlayers, args.actor_net_layer_width)
    elif args.actor_net_type == "PosePolicyNet":
        actor = PosePolicyNet(size_s, size_a, args.actor_net_nlayers, args.actor_net_layer_width)
    else:
        raise KeyError(f"Actor network type {args.actor_net_type} not supported")
    path = Path(__file__).parent / "saves" / args.env
    actor.load_state_dict(torch.load(path / "actor.pt"))
    with open(path / "state_norm.pkl", "rb") as f:
        state_norm = pickle.load(f)
    with open(path / "goal_norm.pkl", "rb") as f:
        goal_norm = pickle.load(f)
    render = True
    
    state, goal, _ = unwrap_obs(env.reset(goal=init_goal))
    done = False
    t = 0
    early_stop = 0
    while not done:
        try:
            env.render()
            time.sleep(0.04)  # Gym operates on 25 Hz
        except mujoco_py.cymj.GlfwError:
            print("No display available, rendering disabled")
            render = False
        state, goal = state_norm(state), goal_norm(goal)
        state = torch.as_tensor(state, dtype=torch.float32)
        goal = torch.as_tensor(goal, dtype=torch.float32)
        with torch.no_grad():
            action = actor(torch.cat([state, goal])).numpy()
        next_obs, reward, done, info = env.step(action)
        state, goal, _ = unwrap_obs(next_obs)

        early_stop = (early_stop + 1) if not reward else 0
        if early_stop == 10:
            break
    print("Success: " + str(info["is_success"]))
