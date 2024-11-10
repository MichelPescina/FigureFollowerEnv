from gymnasium.envs.registration import register
from FigFollowerEnv.env.FigFollowerV0 import FigFollowerV0
from FigFollowerEnv.get_assets_path import get_assets_path

register(
    id="FigFollowerEnv/FigFollower-v0",
    entry_point="FigFollowerEnv.env.FigFollowerV0:FigFollowerV0",
)