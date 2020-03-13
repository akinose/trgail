from gym.envs.mujoco.mujoco_env import MujocoEnv
# ^^^^^ so that user gets the correct error
# message if mujoco is not installed correctly
from gym.envs.mujoco.ant import AntEnv
from gym.envs.mujoco.half_cheetah import HalfCheetahEnv
from gym.envs.mujoco.hopper import HopperEnv
from gym.envs.mujoco.walker2d import Walker2dEnv
from gym.envs.mujoco.humanoid import HumanoidEnv
from gym.envs.mujoco.inverted_pendulum import InvertedPendulumEnv
from gym.envs.mujoco.inverted_double_pendulum import InvertedDoublePendulumEnv
from gym.envs.mujoco.reacher import ReacherEnv
from gym.envs.mujoco.swimmer import SwimmerEnv
from gym.envs.mujoco.humanoidstandup import HumanoidStandupEnv

# Manipulation
from gym.envs.mujoco.pusher import PusherEnv
from gym.envs.mujoco.thrower import ThrowerEnv
from gym.envs.mujoco.striker import StrikerEnv
from gym.envs.mujoco.pusher_goal import PusherGoalEnv
from gym.envs.mujoco.thrower_goal import ThrowerGoalEnv
from gym.envs.mujoco.striker_goal import StrikerGoalEnv

# Base class
from gym.envs.mujoco.base_env import BaseEnv


# Jaco
from gym.envs.mujoco.jaco import JacoEnv
from gym.envs.mujoco.jaco_pick import JacoPickEnv
from gym.envs.mujoco.jaco_catch import JacoCatchEnv
from gym.envs.mujoco.jaco_toss import JacoTossEnv
from gym.envs.mujoco.jaco_hit import JacoHitEnv
from gym.envs.mujoco.jaco_keep_pick import JacoKeepPickEnv
from gym.envs.mujoco.jaco_keep_catch import JacoKeepCatchEnv
from gym.envs.mujoco.jaco_serve import JacoServeEnv
