"""Test a previously trained agent on an OpenAI gym environment.

The MuJoCoVideoRecorder is a wrapper around OpenAI's gym VideoRecorder.
"""

from dextgen.parse_args import parse_args
from dextgen.rollout.rollout import repeated_rollout

if __name__ == "__main__":
    args = parse_args()
    repeated_rollout(args)