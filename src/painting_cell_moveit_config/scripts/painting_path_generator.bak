#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class PaintingPathGenerator(Node):

    def __init__(self):
        super().__init__('painting_path_generator')

        self.publisher = self.create_publisher(
            MarkerArray,
            '/painting_paths',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_paths)

        self.get_logger().info(
            'Automotive painting path generator started.'
        )

    def create_path(self, marker_id, name, points, rgb):

        marker = Marker()

        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.025

        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1.0

        for x, y, z in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)

        return marker

    # Generate parallel horizontal painting passes
    def generate_horizontal_passes(
        self, x_start, x_end,
        y_start, y_end,
        z, passes
    ):

        points = []

        for i in range(passes):

            y = y_start + i * (y_end - y_start) / (passes - 1)

            if i % 2 == 0:
                points.append((x_start, y, z))
                points.append((x_end, y, z))
            else:
                points.append((x_end, y, z))
                points.append((x_start, y, z))

        return points

    # Generate vertical painting passes on a side panel
    def generate_vertical_passes(
        self, x_start, x_end,
        y, z_bottom, z_top,
        passes
    ):

        points = []

        for i in range(passes):

            x = x_start + i * (x_end - x_start) / (passes - 1)

            if i % 2 == 0:
                points.append((x, y, z_bottom))
                points.append((x, y, z_top))
            else:
                points.append((x, y, z_top))
                points.append((x, y, z_bottom))

        return points

    # Generate vertical passes on the rear surface
    def generate_rear_passes(
        self, x, y_start, y_end,
        z_bottom, z_top, passes
    ):

        points = []

        for i in range(passes):

            y = y_start + i * (y_end - y_start) / (passes - 1)

            if i % 2 == 0:
                points.append((x, y, z_bottom))
                points.append((x, y, z_top))
            else:
                points.append((x, y, z_top))
                points.append((x, y, z_bottom))

        return points

    def publish_paths(self):

        markers = MarkerArray()

        # ==================================
        # 1. BONNET
        # ==================================

        bonnet = self.generate_horizontal_passes(
            -2.05,
            -0.85,
            -0.65,
            0.65,
            1.03,
            5
        )

        markers.markers.append(
            self.create_path(
                0,
                'bonnet',
                bonnet,
                (1.0, 0.8, 0.0)
            )
        )

        # ==================================
        # 2. ROOF
        # ==================================

        roof = self.generate_horizontal_passes(
            -0.65,
            0.85,
            -0.55,
            0.55,
            1.52,
            5
        )

        markers.markers.append(
            self.create_path(
                1,
                'roof',
                roof,
                (0.0, 0.3, 1.0)
            )
        )

        # ==================================
        # 3. NEAR-SIDE DOORS
        # ==================================

        doors = self.generate_vertical_passes(
            -0.75,
            1.35,
            -1.08,
            0.55,
            1.15,
            7
        )

        markers.markers.append(
            self.create_path(
                2,
                'doors',
                doors,
                (0.0, 1.0, 0.2)
            )
        )

        # ==================================
        # 4. REAR BODY
        # ==================================

        rear = self.generate_rear_passes(
            2.48,
            -0.65,
            0.65,
            0.35,
            1.05,
            5
        )

        markers.markers.append(
            self.create_path(
                3,
                'rear_body',
                rear,
                (1.0, 0.0, 0.0)
            )
        )

        # 5. FAR-SIDE DOORS (opposite side of the car)
        doors_far = self.generate_vertical_passes(-0.75, 1.35, 1.08, 0.55, 1.15, 7)
        markers.markers.append(
            self.create_path(4, 'doors_far', doors_far, (0.0, 0.8, 0.8)))

        # 6. FRONT FACE
        front = self.generate_rear_passes(-2.48, -0.65, 0.65, 0.35, 1.05, 5)
        markers.markers.append(
            self.create_path(5, 'front', front, (1.0, 0.5, 0.0)))

        self.publisher.publish(markers)


def main(args=None):

    rclpy.init(args=args)

    node = PaintingPathGenerator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
