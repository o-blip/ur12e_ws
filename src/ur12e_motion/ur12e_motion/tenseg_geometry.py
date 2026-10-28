"""Define and visualize the tensegrity prism geometry."""

import numpy as np


def tensegrity_nodes(radius=4.0 * np.sqrt(3.0) * 0.0254, height=4.0 * np.sqrt(3.0) * 0.0254) -> dict[int, np.ndarray]:
    """Return node positions expressed in the prism frame."""
    angles_deg = {
        1: 90,
        2: 210,
        3: 330,
        4: 120,
        5: 240,
        6: 360,
    }

    nodes = {}

    for node, angle_deg in angles_deg.items():
        angle = np.deg2rad(angle_deg)
        z = height / 2 if node <= 3 else -height / 2

        nodes[node] = np.array([
            radius * np.cos(angle),
            radius * np.sin(angle),
            z,
        ])

    return nodes


def connectivity_matrix(
    connectivity_indices: np.ndarray,
    node_count: int,
) -> np.ndarray:
    """Convert one-based member endpoint indices to a connectivity matrix."""
    identity = np.eye(node_count, dtype=int)
    start_nodes = connectivity_indices[:, 0] - 1
    end_nodes = connectivity_indices[:, 1] - 1

    return identity[end_nodes] - identity[start_nodes]


def main():
    """Print and plot the tensegrity nodes in three dimensions."""
    import matplotlib.pyplot as plt

    INCH_TO_METER = 0.0254

    radius = 4.0 * np.sqrt(3.0) * INCH_TO_METER
    height = radius

    nodes = tensegrity_nodes(radius, height)

    for node, position in nodes.items():
        print(f'Node {node}: {position}')

    node_matrix = np.column_stack([
        nodes[node] for node in sorted(nodes)
    ])

    C_s_in = np.array([
        [1, 2],
        [2, 3],
        [3, 1],
        [1, 4],
        [2, 5],
        [3, 6],
        [4, 5],
        [5, 6],
        [6, 4],
    ])
    C_b_in = np.array([
        [1, 5],
        [2, 6],
        [3, 4],
    ])

    C_s = connectivity_matrix(C_s_in, node_matrix.shape[1])
    C_b = connectivity_matrix(C_b_in, node_matrix.shape[1])

    string_matrix = node_matrix @ C_s.T
    bar_matrix = node_matrix @ C_b.T

    print(f'Node matrix (shape {node_matrix.shape}):\n{node_matrix}')
    print(f'String connectivity index matrix:\n{C_s_in}')
    print(f'String connectivity matrix:\n{C_s}')
    print(
        f'String matrix (shape {string_matrix.shape}):\n'
        f'{string_matrix}'
    )
    print(f'Bar connectivity index matrix:\n{C_b_in}')
    print(f'Bar connectivity matrix:\n{C_b}')
    print(f'Bar matrix (shape {bar_matrix.shape}):\n{bar_matrix}')

    figure = plt.figure()
    axes = figure.add_subplot(111, projection='3d')
    axes.scatter(
        node_matrix[0, :],
        node_matrix[1, :],
        node_matrix[2, :],
        s=60,
        color='black',
        label='Nodes',
    )

    for member, (start_node, end_node) in enumerate(C_s_in):
        endpoints = node_matrix[:, [start_node - 1, end_node - 1]]
        axes.plot(
            endpoints[0, :],
            endpoints[1, :],
            endpoints[2, :],
            color='tab:blue',
            linewidth=2,
            label='Strings' if member == 0 else None,
        )

    for member, (start_node, end_node) in enumerate(C_b_in):
        endpoints = node_matrix[:, [start_node - 1, end_node - 1]]
        axes.plot(
            endpoints[0, :],
            endpoints[1, :],
            endpoints[2, :],
            color='tab:red',
            linewidth=4,
            label='Bars' if member == 0 else None,
        )

    for node, (x, y, z) in nodes.items():
        axes.text(x, y, z, f'  {node}', fontsize=10)

    axes.set(
        title='Tensegrity Nodes',
        xlabel='X (m)',
        ylabel='Y (m)',
        zlabel='Z (m)',
    )
    axes.set_box_aspect((1, 1, 1))
    axes.legend()
    figure.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
