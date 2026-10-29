"""Cartesian feed paths over a stationary D-bar, expressed in base_link."""

import numpy as np

from dbar_tensegrity.geometry import C_s_in, node1, node2, node3, node4


# %% Fixture placement (meters)
NODE1_BASE = (0.150, 0.50, 0.5) # Base position of node 1 in meters relative to base_link
FEED_HEIGHT = 0.01 # height above node for string-feed in meters


# %% Cartesian path generation
# Half turn about X: tool +Z points along base -Z.
TOOL_ROTATION = np.diag([1.0, -1.0, -1.0])


def generate_string_paths(
    node1_base=NODE1_BASE,
    feed_height=FEED_HEIGHT,
    steps=101,
):
    """Return separate arrays of tool poses for each string in C_s_in.

    Each array has shape (steps, 4, 4), including both endpoints. The D-bar
    axes align with base_link. The feed tip is provisionally at tool0, with
    tool +Z pointing downward (180 degrees about base X).

    These are geometric waypoints without timing or approach motions.
    Transfers are added separately by generate_winding_sequence.
    """
    if not isinstance(steps, int) or isinstance(steps, bool) or steps < 2:
        raise ValueError('steps must be an integer of at least 2')
    
    origin = np.asarray(node1_base, dtype=float)
    if origin.shape != (3,) or not np.all(np.isfinite(origin)):
        raise ValueError('node1_base must contain three finite coordinates')
    if not np.isfinite(feed_height) or feed_height < 0:
        raise ValueError('feed_height must be finite and nonnegative')

    local_nodes = np.asarray([node1, node2, node3, node4], dtype=float)
    feed_nodes = local_nodes - local_nodes[0] + origin
    feed_nodes[:, 2] += feed_height

    paths = []
    for start, end in C_s_in:
        poses = np.repeat(np.eye(4)[None, :, :], steps, axis=0)
        poses[:, :3, :3] = TOOL_ROTATION
        poses[:, :3, 3] = np.linspace(
            feed_nodes[start - 1], feed_nodes[end - 1], steps,
        )
        paths.append(poses)
    return paths


# %% Ordered winding and transfer segments
def generate_winding_sequence(
    node1_base=NODE1_BASE,
    feed_height=FEED_HEIGHT,
    steps=101,
):
    """Return ordered (kind, poses) pairs, inserting straight transfers.

    For the current geometry: wind 1->3, transfer 3->2, wind 2->4.
    Transfers keep the same feed height and orientation. Segment boundaries
    retain shared endpoints so a cut-and-resume pause can be added later.
    This function does not implement pauses, timing, or robot execution.
    """
    string_paths = generate_string_paths(node1_base, feed_height, steps)
    sequence = []
    for index, path in enumerate(string_paths):
        if index:
            transfer = np.repeat(np.eye(4)[None, :, :], steps, axis=0)
            transfer[:, :3, :3] = TOOL_ROTATION
            transfer[:, :3, 3] = np.linspace(
                string_paths[index - 1][-1, :3, 3],
                path[0, :3, 3],
                steps,
            )
            sequence.append(('transfer', transfer))
        sequence.append(('wind', path))
    return sequence
