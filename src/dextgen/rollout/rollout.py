import time
import os
from pathlib import Path
from typing import Optional, Tuple, Any
import logging
import pickle

import mujoco_py
import gym
from gym.wrappers.monitoring.video_recorder import VideoRecorder

import torch
from dextgen.mp_rl.core.utils import unwrap_obs
from dextgen.mp_rl.core.actor import PosePolicyNet, DDP

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

    def capture_frame(self) -> Any:
        """Capture a Mujoco frame.

        Returns:
            The frame in the previously specified format.
        """
        if self.resolution is None:
            return super().capture_frame()
        if not self.functional or self._closed:
            return
        # For errors with OpenGL see https://github.com/openai/mujoco-py/issues/187
        # Fix: $ unset LD_PRELOAD
        frame = self.env.render("rgb_array", width=self.resolution[0], height=self.resolution[1])
        if frame is None:
            if self._async:
                return
            self.broken = True
        else:
            self.last_frame = frame
            self._encode_image_frame(frame)

def create_logger(args):
    logger = logging.getLogger("GymTestScript")
    loglvls = {
        "DEBUG": logging.DEBUG,
        "INFO": logging.INFO,
        "WARN": logging.WARN,
        "ERROR": logging.ERROR
    }
    logging.basicConfig()
    logging.getLogger().setLevel(loglvls[args.loglvl])

    return logger

def create_actor(env, args, saves_path=os.getcwd()+"/saves"):
    size_g = len(env.observation_space["desired_goal"].low)
    size_s = len(env.observation_space["observation"].low) + size_g
    size_a = len(env.action_space.low)

    if args.actor_net_type == "DDP":
        actor = DDP(size_s, size_a, args.actor_net_nlayers, args.actor_net_layer_width)
    elif args.actor_net_type == "PosePolicyNet":
        actor = PosePolicyNet(size_s, size_a, args.actor_net_nlayers, args.actor_net_layer_width)
    else:
        raise KeyError(f"Actor network type {args.actor_net_type} not supported")
    
    path = Path(saves_path+"/"+args.env)
    actor.load_state_dict(torch.load(path / "actor.pt"))

    with open(path / "state_norm.pkl", "rb") as f:
        state_norm = pickle.load(f)
    with open(path / "goal_norm.pkl", "rb") as f:
        goal_norm = pickle.load(f)
    
    return actor, state_norm, goal_norm

def record_and_render(env, logger, recorder, render):
    if recorder is not None:
        recorder.capture_frame()
    elif render:
        try:
            env.render()
            time.sleep(0.04)  # Gym operates on 25 Hz
        except mujoco_py.cymj.GlfwError:
            logger.warning("No display available, rendering disabled")
            render = False

    return render

def repeated_rollout(args, saves_path=os.getcwd()+"/saves", video_path=os.getcwd()+"/video"):
    logger = create_logger(args)
    env = gym.make(args.env, **args.kwargs) if hasattr(args, "kwargs") else gym.make(args.env)
    actor, state_norm, goal_norm = create_actor(env, args, saves_path)
    
    success = 0.
    render = args.render == "y"
    record = args.record == "y"

    if record:
        path = Path(video_path+"/"+args.env + ".mp4")
        recorder = MujocoVideoRecorder(env, path=str(path), resolution=(1920, 1080))
        logger.info("Recording video, environment rendering disabled")
    else:
        recorder = None

    for i in range(args.ntests):
        success += rollout(env, state_norm, goal_norm, recorder, actor, logger, render)

    if record:
        recorder.close()
        (Path(video_path+"/"+args.env+".meta.json")).unlink()  # Delete metafile
    if render:
        env.destroy_window()

    logger.info(f"Agent success rate: {success/args.ntests:.2f}")

def rollout(env, state_norm, goal_norm, recorder, actor, logger, render):
    state, goal, _ = unwrap_obs(env.reset())
    render = record_and_render(env, logger, recorder, render)

    done = False
    early_stop = 0
    
    while not done:
        action = get_action(actor, state, state_norm, goal, goal_norm)
        next_obs, reward, done, info = env.step(action)
        state, goal, _ = unwrap_obs(next_obs)
        render = record_and_render(env, logger, recorder, render)

        early_stop = (early_stop + 1) if not reward else 0
        
        if early_stop == 10:
            break

    return info["is_success"]

def get_action(actor, state, state_norm, goal, goal_norm):
    state, goal = state_norm(state), goal_norm(goal)
    state = torch.as_tensor(state, dtype=torch.float32)
    goal = torch.as_tensor(goal, dtype=torch.float32)
    with torch.no_grad():
        action = actor(torch.cat([state, goal])).numpy()

    return action
