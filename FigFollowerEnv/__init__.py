from gymnasium.envs.registration import register
from FigFollowerEnv.env.FigFollowerV0 import FigFollowerV0
from FigFollowerEnv.env.FigFollowerV1 import FigFollowerV1
from FigFollowerEnv.get_assets_path import get_assets_path

register(
    id="FigFollowerEnv-v0",
    entry_point="FigFollowerEnv.env.FigFollowerV0:FigFollowerV0",
)

register(
    id="FigFollowerEnv-v1",
    entry_point="FigFollowerEnv.env.FigFollowerV1:FigFollowerV1",
)