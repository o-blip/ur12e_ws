'''
TODO: clean up step 2 

Experimental setup:

    Experimental frame = World frame centered on robot base
    F = String feed point (e.g. [2,2,2] feet from the base of the robot)
    N = [n1 n2 n3 n4 n5 n6] nodes of the tensegrity
    Bars:
        b1 = n5 - n1
        b2 = n6 - n2
        b3 = n4 - n3

        Therefore bars are defined from top node → bottom node.

Objective: Wind the strings of the tensegrity using the robot arm. For a 3-bar, there are 9 strings

Cartesian trajectory:
    1. Wind the top nodes
        Requirements: Send n_i to F and keep bar b_i pointing up and prism z-axis pointing away from the base
        Example for node 1:
            -send n1 to F
            -rotate the prism such that n5 - n1 aligned with +Z
            -send n2 to F by rotating through the prism z-axis
    2. Cross string (1-4)
        Requirements: keep the active string segment in a horizontal plane
        Example for string 1-4:
            -n1 starts at F
            -orient the prism so:
                (n4 - n1) · Z = 0   
            -then move the prism so n4 reaches F
                while maintaining that horizontal-string condition
            -rotate so that bar 3-4 points up
    3. Wind the bottom nodes, wind 1 past the initial node
        Same as in 1. but the bars should be pointing down as a result of how we define them
        Repeat over one string to get to the starting node of another vertical string
    4. Wind another vertical string
        Repeat 2.
    5. Wind one of the top strings to the last remaining vertical string start node
    6. Wind the remaining vertical node

'''
from scipy.spatial.transform import Rotation
import numpy as np
from tenseg_geometry import tensegrity_nodes
from animation import animate_poses




def node_world_position(node_P, R_B_P, p_B_P):
    '''Transforms local node positions to world node positions.
    node_P: position of node in prism frame
    R_B_P: rotation from prism frame to world frame
    p_B_P: translation from prism frame to world frame'''
    return R_B_P @ node_P + p_B_P

def homogeneous_transform(R, p):
    '''Creates a homogeneous transform from a rotation matrix and translation vector'''
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def planar_node_angle(node_a, node_b):
    """Signed XY-plane angle from node_a to node_b."""
    a = node_a[:2]
    b = node_b[:2]

    return np.arctan2(
        a[0] * b[1] - a[1] * b[0],
        np.dot(a, b)
    )

def bar_to_vertical(bar_P, R_B_P):
    """Returns a SciPy Rotation object of the needed rotation that aligns the bar with 
    the world +Z axis."""
    bar_direction = R_B_P @ bar_P # bar world direction
    bar_direction /= np.linalg.norm(bar_direction)

    world_up = np.array([0.0, 0.0, 1.0])

    R_align, _ = Rotation.align_vectors(
        [world_up],
        [bar_direction]
    )

    return R_align


def incremental_rotation_about_node(
    R_delta,
    R_B_P_start,
    fixed_node_P,
    fixed_point_B,
    steps=100,
):
    """Apply a world-frame rotation while keeping one node fixed.
    Used to rotate the prism about the feed point"""
    path = []
    delta_rotvec = R_delta.as_rotvec()

    for alpha in np.linspace(0.0, 1.0, steps)[1:]:
        R_step = Rotation.from_rotvec(
            alpha * delta_rotvec
        ).as_matrix()

        R_B_P = R_step @ R_B_P_start
        p_B_P = fixed_point_B - R_B_P @ fixed_node_P

        path.append(homogeneous_transform(R_B_P, p_B_P))

    return path, R_B_P, p_B_P


def generate_cross_string_path(
    nodes,
    start_pose,
    feed_point,
    start_node,
    end_node,
    bar_other_node,
    steps=100,
):
    """Generate the path for one cross string."""
    path = [start_pose.copy()]
    # Initial prism orientation and position
    R_B_P = start_pose[:3, :3].copy()
    p_B_P = start_pose[:3, 3].copy()

    # Phase 1: rotate until the cross string is horizontal.
    string_P = nodes[end_node] - nodes[start_node]
    string_B = R_B_P @ string_P
    string_B /= np.linalg.norm(string_B)
    horizontal_string_B = string_B.copy()
    horizontal_string_B[2] = 0.0
    horizontal_norm = np.linalg.norm(horizontal_string_B)
    horizontal_string_B /= horizontal_norm 
    R_horizontal, _ = Rotation.align_vectors(
        [horizontal_string_B],
        [string_B],
    )

    new_poses, R_B_P, p_B_P = incremental_rotation_about_node(
        R_delta=R_horizontal,
        R_B_P_start=R_B_P,
        fixed_node_P=nodes[start_node],
        fixed_point_B=feed_point,
        steps=steps,
    )
    path.extend(new_poses)

    # Phase 2: translate until the end node reaches the feed point.
    end_node_B = node_world_position(
        nodes[end_node],
        R_B_P,
        p_B_P,
    )
    delta_p = feed_point - end_node_B # how much to translate the prism in world frame to get the end node to the feed point
    p_B_P_start = p_B_P.copy()

    for alpha in np.linspace(0.0, 1.0, steps)[1:]:
        p_B_P = p_B_P_start + alpha * delta_p
        path.append(homogeneous_transform(R_B_P, p_B_P)) # new pose only updates the translation, not the rotation

    # Phase 3: rotate about the end node until its bar points upward.
    bar_P = nodes[bar_other_node] - nodes[end_node]
    R_vertical = bar_to_vertical(bar_P, R_B_P)

    new_poses, R_B_P, p_B_P = incremental_rotation_about_node(
        R_delta=R_vertical,
        R_B_P_start=R_B_P,
        fixed_node_P=nodes[end_node],
        fixed_point_B=feed_point,
        steps=steps,
    )
    path.extend(new_poses)

    return path


def initial_winding_pose(nodes, feed_point, start_node, bar_other_node):
    """Create an initial pose with one node at the feed point.

    The selected bar points along world +Z and the horizontal projection of
    the prism +Z axis points away from the robot base.
    """
    bar_P = nodes[bar_other_node] - nodes[start_node]
    R1 = bar_to_vertical(bar_P, np.eye(3)).as_matrix() # send bar to vertical

    away = feed_point.copy()
    away[2] = 0.0
    away /= np.linalg.norm(away)

    z_P = np.array([0.0, 0.0, 1.0])
    current_z_B = R1 @ z_P
    current_z_B[2] = 0.0
    current_z_B /= np.linalg.norm(current_z_B)

    yaw = planar_node_angle(current_z_B, away)
    R_yaw = Rotation.from_euler('z', yaw).as_matrix() # rotation to point prism z-axis away from the robot base
    R_B_P = R_yaw @ R1 # combined rotation

    p_B_P = feed_point - R_B_P @ nodes[start_node] # send start node to feed
    return homogeneous_transform(R_B_P, p_B_P)


def generate_planar_string_path(
    nodes,
    start_pose,
    start_node,
    end_node,
    steps=100,
):
    """Rotate about prism +Z so the end node reaches the start node."""
    path = [start_pose.copy()]
    R_B_P_start = start_pose[:3, :3].copy()
    p_B_P = start_pose[:3, 3].copy()

    # The end node must rotate into the start node's current position.
    theta = planar_node_angle(nodes[end_node], nodes[start_node])

    for alpha in np.linspace(0.0, 1.0, steps)[1:]:
        R_step_P = Rotation.from_euler('z', alpha * theta).as_matrix()
        R_B_P = R_B_P_start @ R_step_P
        path.append(homogeneous_transform(R_B_P, p_B_P))
    return path



#### Constants
INCH_TO_METER = 0.0254 # UR uses meters for units, but the prism is defined in inches

# position of the string feed point in the robot base frame
feed_point = np.array([
    24.0,
    0.0,
    24.0,
]) * INCH_TO_METER

nodes = tensegrity_nodes() # generates the prism centered at origin

# Cartesian path generation
initial_pose = initial_winding_pose(
    nodes,
    feed_point,
    start_node=1,
    bar_other_node=5,
)

top_path = [initial_pose]
for start_node, end_node in ((1, 2), (2, 3), (3, 1)):
    string_path = generate_planar_string_path(
        nodes,
        top_path[-1],
        start_node,
        end_node,
    )
    top_path.extend(string_path[1:])

cross_14_path = generate_cross_string_path(
    nodes,
    top_path[-1],
    feed_point,
    start_node=1,
    end_node=4,
    bar_other_node=3,
)

bottom_path = [cross_14_path[-1]]
for start_node, end_node in ((4, 5), (5, 6), (6, 4), (4,5)):
    # ends on node 5, which is the start node of the next vertical string
    string_path = generate_planar_string_path(
        nodes,
        bottom_path[-1],
        start_node,
        end_node,
    )
    bottom_path.extend(string_path[1:])

cross_52_path = generate_cross_string_path(
    nodes,
    bottom_path[-1],
    feed_point,
    start_node=5,
    end_node=2,
    bar_other_node=6,
)

repeat_23 = generate_planar_string_path(
    nodes,
    cross_52_path[-1],
    start_node=2,
    end_node=3,
)

cross_36_path = generate_cross_string_path(
    nodes,
    repeat_23[-1],
    feed_point,
    start_node=3,
    end_node=6,
    bar_other_node=4,
)


if __name__ == "__main__":
    complete_path = top_path + cross_14_path[1:]+ bottom_path[1:]+ cross_52_path[1:]+ repeat_23[1:]+ cross_36_path[1:]
    animate_poses(nodes, complete_path, feed_point=feed_point)
