import numpy as np
import rclpy

from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from ur12e_motion.tenseg_geometry import tensegrity_nodes


INCH_TO_METER = 0.0254

# Change these if your physical struts connect different nodes.
BAR_PAIRS = [
    (1, 5),
    (2, 6),
    (3, 4),
]


class WindingVisualizer(Node):
    def __init__(self):
        super().__init__('winding_visualizer')

        self.publisher = self.create_publisher(
            MarkerArray,
            '/winding_markers',
            10,
        )

        radius = 4.0 * np.sqrt(3.0) * INCH_TO_METER
        height = radius

        # Node coordinates expressed in prism frame {P}.
        self.nodes_p = tensegrity_nodes(radius, height)

        # Prism center is 8 inches along +z of tool0.
        self.tool_to_prism_offset = np.array([
            0.0,
            0.0,
            8.0 * INCH_TO_METER,
        ])

        # Feed point expressed in base_link.
        self.feed_position = np.array([
            24.0 * INCH_TO_METER,
            0.0,
            24.0 * INCH_TO_METER,
        ])

        # Re-publish so RViz receives the markers even if opened later.
        self.timer = self.create_timer(0.5, self.publish_markers)

    @staticmethod
    def make_point(position: np.ndarray) -> Point:
        point = Point()
        point.x = float(position[0])
        point.y = float(position[1])
        point.z = float(position[2])
        return point

    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Fixed string-feed point.
        feed = Marker()
        feed.header.frame_id = 'base_link'
        feed.header.stamp = stamp
        feed.ns = 'feed'
        feed.id = 0
        feed.type = Marker.SPHERE
        feed.action = Marker.ADD

        feed.pose.position = self.make_point(self.feed_position)
        feed.pose.orientation.w = 1.0

        feed.scale.x = 0.04
        feed.scale.y = 0.04
        feed.scale.z = 0.04

        feed.color.r = 1.0
        feed.color.a = 1.0

        markers.markers.append(feed)

        # Vertical stand from the ground to the feed point.
        stand = Marker()
        stand.header.frame_id = 'base_link'
        stand.header.stamp = stamp
        stand.ns = 'feed_stand'
        stand.id = 1
        stand.type = Marker.CYLINDER
        stand.action = Marker.ADD

        # A cylinder's position is at its center.
        stand.pose.position.x = float(self.feed_position[0])
        stand.pose.position.y = float(self.feed_position[1])
        stand.pose.position.z = float(self.feed_position[2] / 2.0)
        stand.pose.orientation.w = 1.0

        # Cylinder diameter and height.
        stand.scale.x = 0.03
        stand.scale.y = 0.03
        stand.scale.z = float(self.feed_position[2])

        stand.color.r = 0.5
        stand.color.g = 0.5
        stand.color.b = 0.5
        stand.color.a = 1.0

        markers.markers.append(stand)

        # Transform nodes from prism coordinates into tool0 coordinates.
        nodes_e = {
            number: position + self.tool_to_prism_offset
            for number, position in self.nodes_p.items()
        }

        # Node spheres and number labels.
        for number, position in nodes_e.items():
            node = Marker()
            node.header.frame_id = 'tool0'
            node.header.stamp = stamp
            node.frame_locked = True
            node.ns = 'prism_nodes'
            node.id = number
            node.type = Marker.SPHERE
            node.action = Marker.ADD

            node.pose.position = self.make_point(position)
            node.pose.orientation.w = 1.0

            node.scale.x = 0.025
            node.scale.y = 0.025
            node.scale.z = 0.025

            node.color.g = 1.0
            node.color.a = 1.0

            markers.markers.append(node)

            label = Marker()
            label.header.frame_id = 'tool0'
            label.header.stamp = stamp
            label.frame_locked = True
            label.ns = 'node_labels'
            label.id = 100 + number
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label_position = position.copy()
            label_position[2] += 0.035

            label.pose.position = self.make_point(label_position)
            label.pose.orientation.w = 1.0

            label.scale.z = 0.03
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(number)

            markers.markers.append(label)

        # Three tensegrity bars.
        bars = Marker()
        bars.header.frame_id = 'tool0'
        bars.header.stamp = stamp
        bars.frame_locked = True
        bars.ns = 'prism_bars'
        bars.id = 200
        bars.type = Marker.LINE_LIST
        bars.action = Marker.ADD

        bars.scale.x = 0.012
        bars.color.b = 1.0
        bars.color.a = 1.0

        for first_node, second_node in BAR_PAIRS:
            bars.points.append(self.make_point(nodes_e[first_node]))
            bars.points.append(self.make_point(nodes_e[second_node]))

        markers.markers.append(bars)

        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)

    node = WindingVisualizer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()