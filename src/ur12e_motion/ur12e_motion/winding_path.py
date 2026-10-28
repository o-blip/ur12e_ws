'''import numpy as np
from ur12e_motion.tenseg_geometry import tensegrity_nodes
from scipy.spatial.transform import Rotation

INCH_TO_METER = 0.0254

def rotation_about_z(angle: float) -> np.ndarray:
    """Return a 3x3 rotation matrix about the prism z-axis."""

    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ])

def angle_to_feed(
    start_node: np.ndarray,
    target_node: np.ndarray,
) -> float:
    """
    Find the z-axis rotation that moves target_node to the original
    position of start_node.

    Node coordinates are expressed in the prism frame.
    """

    start_xy = start_node[:2]
    target_xy = target_node[:2]

    dot = np.dot(target_xy, start_xy)

    cross = (
        target_xy[0] * start_xy[1]
        - target_xy[1] * start_xy[0]
    )

    return np.arctan2(cross, dot)

def initial_prism_position(
    feed_position: np.ndarray,
    prism_rotation: np.ndarray,
    start_node: np.ndarray,
) -> np.ndarray:
    """Place start_node directly at the fixed feed point."""

    return feed_position - prism_rotation @ start_node

def prism_winding_pose(
    prism_position: np.ndarray,
    initial_rotation: np.ndarray,
    angle: float,
) -> np.ndarray:
    """
    Return the desired prism pose for one winding angle.

    The prism center stays fixed while the prism rotates about its z-axis.
    """

    prism_rotation = initial_rotation @ rotation_about_z(angle)

    return make_transform(
        rotation=prism_rotation,
        position=prism_position,
    )

def make_transform(rotation: np.ndarray, position: np.ndarray) -> np.ndarray:
    """Create a 4x4 homogeneous transform."""

    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position

    return transform

def tool_pose_from_prism_pose(
    T_B_P: np.ndarray,
    T_E_P: np.ndarray,
) -> np.ndarray:
    """
    Compute the required tool pose from the desired prism pose.

    T_B_P: prism frame {P} expressed in base frame {B}
    T_E_P: prism frame {P} expressed in tool frame {E}
    """

    return T_B_P @ np.linalg.inv(T_E_P)

def transform_to_pose(T: np.ndarray):
    """
    Convert a 4x4 transform into position and quaternion.

    Quaternion order is [x, y, z, w], matching ROS.
    """

    position = T[:3, 3]

    rotation_matrix = T[:3, :3]
    quaternion = Rotation.from_matrix(rotation_matrix).as_quat()

    return position, quaternion

def generate_triangle_winding_poses(
    nodes: dict[int, np.ndarray],
    sequence: list[int],
    feed_position: np.ndarray,
    initial_prism_rotation: np.ndarray,
    T_E_P: np.ndarray,
) -> list[np.ndarray]:
    """Generate the required tool transforms for a triangle winding."""

    prism_position = initial_prism_position(
        feed_position,
        initial_prism_rotation,
        nodes[sequence[0]],
    )

    tool_poses = []
    cumulative_angle = 0.0

    # Include the initial pose with the first node at the feed point.
    initial_prism_pose = prism_winding_pose(
        prism_position,
        initial_prism_rotation,
        cumulative_angle,
    )

    tool_poses.append(
        tool_pose_from_prism_pose(initial_prism_pose, T_E_P)
    )

    # Generate each following winding pose.
    for current_node, next_node in zip(sequence[:-1], sequence[1:]):
        delta_angle = angle_to_feed(
            nodes[current_node],
            nodes[next_node],
        )

        cumulative_angle += delta_angle

        T_B_P = prism_winding_pose(
            prism_position,
            initial_prism_rotation,
            cumulative_angle,
        )

        tool_poses.append(
            tool_pose_from_prism_pose(T_B_P, T_E_P)
        )

    return tool_poses

def align_vector_to_vector(
    source: np.ndarray,
    target: np.ndarray,
) -> np.ndarray:
    """
    Return a rotation matrix that maps source onto target.

    Both vectors can be given in arbitrary magnitudes.
    """

    source = source / np.linalg.norm(source)
    target = target / np.linalg.norm(target)

    cross = np.cross(source, target)
    dot = np.clip(np.dot(source, target), -1.0, 1.0)
    cross_norm = np.linalg.norm(cross)

    # Vectors already point in the same direction.
    if cross_norm < 1e-10 and dot > 0.0:
        return np.eye(3)

    # Vectors point in opposite directions.
    if cross_norm < 1e-10 and dot < 0.0:
        reference = np.array([1.0, 0.0, 0.0])

        if abs(source[0]) > 0.9:
            reference = np.array([0.0, 1.0, 0.0])

        axis = np.cross(source, reference)
        axis /= np.linalg.norm(axis)

        # Rotation by 180 degrees.
        return 2.0 * np.outer(axis, axis) - np.eye(3)

    skew = np.array([
        [0.0, -cross[2], cross[1]],
        [cross[2], 0.0, -cross[0]],
        [-cross[1], cross[0], 0.0],
    ])

    return (
        np.eye(3)
        + skew
        + skew @ skew * ((1.0 - dot) / cross_norm**2)
    )

def initial_winding_rotation(
    nodes: dict[int, np.ndarray],
    lower_node: int,
    upper_node: int,
    feed_position: np.ndarray,
) -> np.ndarray:
    """
    Orient the prism so:

    1. The bar from lower_node to upper_node points vertically upward.
    2. The horizontal projection of the prism +z-axis points away
       from the robot.
    """

    bar_direction_p = nodes[upper_node] - nodes[lower_node]
    world_up_b = np.array([0.0, 0.0, 1.0])

    R_align = align_vector_to_vector(
        bar_direction_p,
        world_up_b,
    )

    prism_z_b = R_align @ np.array([0.0, 0.0, 1.0])

    away_from_robot_b = np.array([
        feed_position[0],
        feed_position[1],
        0.0,
    ])

    current_xy = prism_z_b[:2]
    desired_xy = away_from_robot_b[:2]

    current_xy /= np.linalg.norm(current_xy)
    desired_xy /= np.linalg.norm(desired_xy)

    yaw = np.arctan2(
        current_xy[0] * desired_xy[1]
        - current_xy[1] * desired_xy[0],
        np.dot(current_xy, desired_xy),
    )

    return rotation_about_z(yaw) @ R_align

if __name__ == '__main__':
    radius = 4.0 * np.sqrt(3.0) * INCH_TO_METER
    height = radius

    nodes = tensegrity_nodes(radius, height)

    # Fixed string-feed point in the robot base frame.
    feed_position = np.array([
        24.0 * INCH_TO_METER,
        0.0,
        24.0 * INCH_TO_METER,
    ])

    # Initial prism orientation relative to the robot base.
    initial_prism_rotation = np.eye(3)

    # Place node 1 at the feed point.
    prism_position = initial_prism_position(
        feed_position,
        initial_prism_rotation,
        nodes[1],
    )

    # Fixed tool-to-prism transform.
    T_E_P = make_transform(
        rotation=np.eye(3),
        position=np.array([0.0, 0.0, 8.0 * INCH_TO_METER]),
    )

    sequence = [1, 2, 3, 1]

    # Start with node 1 already positioned at the feed point.
    cumulative_angle = 0.0

    for current_node, next_node in zip(sequence[:-1], sequence[1:]):

        # Rotation required to bring the next node to the location
        # currently occupied by the current node.
        delta_angle = angle_to_feed(
            nodes[current_node],
            nodes[next_node],
        )

        cumulative_angle += delta_angle

        T_B_P = prism_winding_pose(
            prism_position,
            initial_prism_rotation,
            cumulative_angle,
        )

        T_B_E = tool_pose_from_prism_pose(T_B_P, T_E_P)
        position, quaternion = transform_to_pose(T_B_E)

        print(f'Position: {position}')
        print(f'Quaternion [x, y, z, w]: {quaternion}')


        print(f'\n{current_node} -> {next_node}')
        print(f'Increment: {np.rad2deg(delta_angle):.2f} degrees')
        print(f'Total rotation: {np.rad2deg(cumulative_angle):.2f} degrees')
        print(T_B_E)'''


import numpy as np
from scipy.spatial.transform import Rotation

from ur12e_motion.tenseg_geometry import tensegrity_nodes


INCH_TO_METER = 0.0254




def transform_to_pose(T):
    """Convert a homogeneous transform to position and ROS quaternion."""
    position = T[:3, 3]
    quaternion = Rotation.from_matrix(T[:3, :3]).as_quat()
    return position, quaternion


def node_world_position(node_P, R_B_P, p_B_P):
    """Transform a prism-frame node position into the base frame."""
    return R_B_P @ node_P + p_B_P


def homogeneous_transform(R, p):
    """Create a homogeneous transform from a rotation and translation."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def planar_node_angle(node_a, node_b):
    """Return the signed XY-plane angle from node_a to node_b."""
    a = node_a[:2]
    b = node_b[:2]

    return np.arctan2(
        a[0] * b[1] - a[1] * b[0],
        np.dot(a, b),
    )


def bar_to_vertical(bar_P, R_B_P):
    """Return the world-frame rotation that aligns a bar with world +Z."""
    bar_direction = R_B_P @ bar_P
    bar_direction /= np.linalg.norm(bar_direction)

    world_up = np.array([0.0, 0.0, 1.0])
    R_align, _ = Rotation.align_vectors(
        [world_up],
        [bar_direction],
    )
    return R_align


def incremental_rotation_about_node(
    R_delta,
    R_B_P_start,
    fixed_node_P,
    fixed_point_B,
    steps=100,
):
    """Apply a world-frame rotation while keeping one node fixed."""
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
    R_B_P = start_pose[:3, :3].copy()
    p_B_P = start_pose[:3, 3].copy()

    # Rotate about the start node until the cross string is horizontal.
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

    # Translate until the end node reaches the feed point.
    end_node_B = node_world_position(
        nodes[end_node],
        R_B_P,
        p_B_P,
    )
    delta_p = feed_point - end_node_B
    p_B_P_start = p_B_P.copy()

    for alpha in np.linspace(0.0, 1.0, steps)[1:]:
        p_B_P = p_B_P_start + alpha * delta_p
        path.append(homogeneous_transform(R_B_P, p_B_P))

    # Rotate about the end node until its attached bar points upward.
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
    """Create the initial pose with one node at the feed point."""
    bar_P = nodes[bar_other_node] - nodes[start_node]
    R1 = bar_to_vertical(bar_P, np.eye(3)).as_matrix()

    away = feed_point.copy()
    away[2] = 0.0
    away /= np.linalg.norm(away)

    z_P = np.array([0.0, 0.0, 1.0])
    current_z_B = R1 @ z_P
    current_z_B[2] = 0.0
    current_z_B /= np.linalg.norm(current_z_B)

    yaw = planar_node_angle(current_z_B, away)
    R_yaw = Rotation.from_euler('z', yaw).as_matrix()
    R_B_P = R_yaw @ R1

    p_B_P = feed_point - R_B_P @ nodes[start_node]
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

    theta = planar_node_angle(nodes[end_node], nodes[start_node])

    for alpha in np.linspace(0.0, 1.0, steps)[1:]:
        R_step_P = Rotation.from_euler('z', alpha * theta).as_matrix()
        R_B_P = R_B_P_start @ R_step_P
        path.append(homogeneous_transform(R_B_P, p_B_P))

    return path


def generate_complete_prism_path(
    nodes=None,
    feed_point=None,
    steps=100,
):
    """Generate the complete nine-string winding path as prism poses."""
    if nodes is None:
        nodes = tensegrity_nodes()

    if feed_point is None:
        feed_point = np.array([24.0, 0.0, 24.0]) * INCH_TO_METER

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
            steps=steps,
        )
        top_path.extend(string_path[1:])

    cross_14_path = generate_cross_string_path(
        nodes,
        top_path[-1],
        feed_point,
        start_node=1,
        end_node=4,
        bar_other_node=3,
        steps=steps,
    )

    bottom_path = [cross_14_path[-1]]
    for start_node, end_node in ((4, 5), (5, 6), (6, 4), (4, 5)):
        string_path = generate_planar_string_path(
            nodes,
            bottom_path[-1],
            start_node,
            end_node,
            steps=steps,
        )
        bottom_path.extend(string_path[1:])

    cross_52_path = generate_cross_string_path(
        nodes,
        bottom_path[-1],
        feed_point,
        start_node=5,
        end_node=2,
        bar_other_node=6,
        steps=steps,
    )

    repeat_23 = generate_planar_string_path(
        nodes,
        cross_52_path[-1],
        start_node=2,
        end_node=3,
        steps=steps,
    )

    cross_36_path = generate_cross_string_path(
        nodes,
        repeat_23[-1],
        feed_point,
        start_node=3,
        end_node=6,
        bar_other_node=4,
        steps=steps,
    )

    return (
        top_path
        + cross_14_path[1:]
        + bottom_path[1:]
        + cross_52_path[1:]
        + repeat_23[1:]
        + cross_36_path[1:]
    )
