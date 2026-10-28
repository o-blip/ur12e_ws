"""Animate a tensegrity prism from a sequence of homogeneous poses."""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


STRING_CONNECTIONS = (
    (1, 2), (2, 3), (3, 1),
    (1, 4), (2, 5), (3, 6),
    (4, 5), (5, 6), (6, 4),
)

BAR_CONNECTIONS = ((1, 5), (2, 6), (3, 4))


def transform_nodes(nodes, T_B_P):
    """Return prism nodes expressed in the base frame for one pose."""
    R_B_P = T_B_P[:3, :3]
    p_B_P = T_B_P[:3, 3]
    return {
        node: R_B_P @ position + p_B_P
        for node, position in nodes.items()
    }


def animate_poses(nodes, poses, feed_point=None, interval_ms=30):
    """Animate prism-frame ``nodes`` through a sequence of ``T_B_P`` poses."""
    if not poses:
        raise ValueError("poses must contain at least one transform")

    poses = [np.asarray(pose, dtype=float) for pose in poses]
    if any(pose.shape != (4, 4) for pose in poses):
        raise ValueError("every pose must be a 4x4 homogeneous transform")

    node_numbers = sorted(nodes)
    all_positions = np.array([
        transform_nodes(nodes, pose)[node]
        for pose in poses
        for node in node_numbers
    ])
    if feed_point is not None:
        all_positions = np.vstack((all_positions, feed_point))

    lower = all_positions.min(axis=0)
    upper = all_positions.max(axis=0)
    center = (lower + upper) / 2.0
    half_range = max(np.max(upper - lower) / 2.0, 0.05) * 1.1

    figure = plt.figure()
    axes = figure.add_subplot(111, projection="3d")
    axes.set(
        title="Tensegrity winding path",
        xlabel="X (m)",
        ylabel="Y (m)",
        zlabel="Z (m)",
        xlim=(center[0] - half_range, center[0] + half_range),
        ylim=(center[1] - half_range, center[1] + half_range),
        zlim=(center[2] - half_range, center[2] + half_range),
    )
    axes.set_box_aspect((1, 1, 1))

    if feed_point is not None:
        axes.scatter(*feed_point, color="tab:green", s=70, marker="x", label="Feed point")

    node_plot = axes.scatter([], [], [], color="black", s=35, label="Nodes")
    string_plots = [
        axes.plot([], [], [], color="tab:blue", linewidth=1.5,
                  label="Strings" if index == 0 else None)[0]
        for index, _ in enumerate(STRING_CONNECTIONS)
    ]
    bar_plots = [
        axes.plot([], [], [], color="tab:red", linewidth=4,
                  label="Bars" if index == 0 else None)[0]
        for index, _ in enumerate(BAR_CONNECTIONS)
    ]
    labels = [axes.text(0, 0, 0, str(node)) for node in node_numbers]
    frame_text = axes.text2D(0.02, 0.95, "", transform=axes.transAxes)
    axes.legend()

    def update(frame):
        nodes_B = transform_nodes(nodes, poses[frame])
        positions = np.array([nodes_B[node] for node in node_numbers])
        node_plot._offsets3d = (positions[:, 0], positions[:, 1], positions[:, 2])

        for label, node in zip(labels, node_numbers):
            label.set_position((nodes_B[node][0], nodes_B[node][1]))
            label.set_3d_properties(nodes_B[node][2], zdir="z")

        for line, (start, end) in zip(string_plots, STRING_CONNECTIONS):
            endpoints = np.vstack((nodes_B[start], nodes_B[end]))
            line.set_data(endpoints[:, 0], endpoints[:, 1])
            line.set_3d_properties(endpoints[:, 2])

        for line, (start, end) in zip(bar_plots, BAR_CONNECTIONS):
            endpoints = np.vstack((nodes_B[start], nodes_B[end]))
            line.set_data(endpoints[:, 0], endpoints[:, 1])
            line.set_3d_properties(endpoints[:, 2])

        frame_text.set_text(f"Pose {frame + 1}/{len(poses)}")
        return [node_plot, *string_plots, *bar_plots, *labels, frame_text]

    animation = FuncAnimation(
        figure,
        update,
        frames=len(poses),
        interval=interval_ms,
        blit=False,
        repeat=True,
    )
    figure.tight_layout()
    plt.show()
    return animation
