"""Project setup file."""
from setuptools import setup, find_packages

setup(name="dextgen",
      version="0.1.0",
      author="Martin Schuck",
      author_email="martin.schuck@tum.de",
      description=("A collection of reinforcement learning modules for grasping."),
      keywords="RL MP DDPG DDP",
      package_dir={'': 'src'},
      packages=find_packages(where="src"),
      include_package_data=True)
