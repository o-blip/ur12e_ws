"""Placeholder for planar D-bar geometry, independent of ROS.

Node positions, bar and string connectivity, and dimensions will be defined
once the intended D-bar arrangement is specified. No model is implemented yet.
"""
from math import cos, sin, pi
import numpy as np

# %% Geometry Constants
L = 0.40; # length of bar in meters
theta = 30 * pi/180; # angle in radians 

# %% Nodes
node1 = (0.0, 0.0, 0.0)
node2 = (L*cos(theta), L*sin(theta), 0.0)
node3 = (2*L*cos(theta), 0.0, 0.0)
node4 = (L*cos(theta), -L*sin(theta), 0.0)

# %% Bars
bar1 = (node1, node2)
bar2 = (node2, node3)
bar3 = (node3, node4)
bar4 = (node4, node1)



# %% Strings
string1 = (node1, node3)
string2 = (node2, node4)


# %% Bar and string connectivity index matrices
C_b_in = np.array([
    [1, 2],
    [2, 3],
    [3, 4],
    [4, 1]
])

C_s_in = np.array([
    [1, 3],
    [2, 4]
])
