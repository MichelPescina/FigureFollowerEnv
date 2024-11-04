from gymnasium.envs.registration import register
from FigFollowerEnv.env.Simulation import FigFollower
from FigFollowerEnv.get_assets_path import get_assets_path

register(
    id="FigFollowerEnv/FigFollower",
    entry_point="FigFollowerEnv.env.Simulation:FigFollower",
)