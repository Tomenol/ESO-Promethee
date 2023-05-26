# General toolbox architecture
from . import common
from . import physics

# Simulation of the environment's physics
from .physics import launchpad
from .physics import environment
from .physics import earth

# Simulation of the rocket's physics
from . import rocket

# Math and reference frame tools
from .common import frames
from .common import quaternion

# quick import features
# environment
from .physics.environment import Environment
from .physics.launchpad import LaunchPad

# rocket
from .rocket import RocketSimulator

# common
from .common.quaternion import Quaternion
