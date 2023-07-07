"""Argument parser helper functions."""
import argparse
import os
import logging

import yaml

import dextgen.envs

logger = logging.getLogger(__name__)


def parse_args(args=None, config_path=os.getcwd()+"/config/experiment_config.yaml") -> argparse.Namespace:
    """Parse arguments for the gym environment and logging levels.

    Returns:
        The parsed arguments as a namespace.
    """
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--env",
                        help="Selects the gym environment",
                        choices=dextgen.envs.available_envs,
                        required=True)
    parser.add_argument('--loglvl',
                        help="Logger levels",
                        choices=["DEBUG", "INFO", "WARN", "ERROR"],
                        default="INFO")
    parser.add_argument("--render",
                        help="Render flag. Only used for testing",
                        choices=["y", "n"],
                        default="y")
    parser.add_argument("--record",
                        help="Record video flag. Only used for testing",
                        choices=["y", "n"],
                        default="n")
    parser.add_argument("--ntests",
                        help="Number of evaluation runs. Only used for testing",
                        default=10,
                        type=int)
    parser.add_argument("--init_random",
                        help="If false, specify initial state and goal",
                        choices=["y", "n"],
                        default="y")
    parser.add_argument("--init_goal",
                        help="Goal for the object",
                        default="0,0,0")
    parser.add_argument("--init_object",
                        help="Initial pose for the object",
                        default="0,0,0,1,0,0,0")
    parser.add_argument("--init_jointangles",
                        help="Joint angles of the robot arm",
                        default="0,0,0,0,0,0,0")
    parser.add_argument("--init_gripper",
                        help="Finger position for ROS",
                        default="0")
    args = parser.parse_args(args=args)
    expand_args(args, config_path=config_path)
    return args


def expand_args(args: argparse.Namespace, config_path=os.getcwd()+"/config/experiment_config.yaml"):
    """Expand the arguments namespace with settings from the main config file.

    Config can be found at './config/experiment_config.yaml'. Each config must be named after their
    gym name.

    Args:
        args: User provided arguments namespace.
    """
    logging.basicConfig(level=logging.INFO)
    with open(config_path, "r") as f:
        config = yaml.load(f, yaml.SafeLoader)

    if "Default" not in config.keys():
        raise KeyError("Config file is missing required entry `Default`!")
    for key, val in config["Default"].items():
        setattr(args, key, val)

    if args.env not in config.keys():
        logger.info(f"No specific config for {args.env} found, using defaults for all settings.")
    else:
        for key, val in config[args.env].items():
            setattr(args, key, val)
