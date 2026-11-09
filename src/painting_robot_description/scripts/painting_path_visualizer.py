#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray


class PaintingPathVisualizer(Node):

    def __init__(self):
        super().__init__('painting_path_visualizer')

        self.subscription = self.create_subscription(
            PoseArray,
            '/painting/trajectory',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            MarkerArray,
            '/painting/path_markers',
            10
        )

        self.get_logger().info('Painting path visualizer started')

    def callback(self, msg):

        marker_array = MarkerArray()

        # Five surfaces, 124 waypoints per surface.
        surface_size = 124

        colors = [
            (1.0, 0.0, 0.0),  # Left side
            (0.0, 1.0, 0.0),  # Right side
            (0.0, 0.0, 1.0),  # Roof
            (1.0, 1.0, 0.0),  # Bonnet
            (1.0, 0.0, 1.0)   # Boot
        ]

        for index in range(5):

            start = index * surface_size
            end = min(start + surface_size, len(msg.poses))

            marker = Marker()

            marker.header = msg.header
            marker.ns = 'painting_surfaces'
            marker.id = index

            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.025

            r, g, b = colors[index]

            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            marker.pose.orientation.w = 1.0

            for pose in msg.poses[start:end]:
                marker.points.append(pose.position)

            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)


def main(args=None):

    rclpy.init(args=args)

    node = PaintingPathVisualizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
