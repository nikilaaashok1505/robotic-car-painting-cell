#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header

import math


class PaintingTrajectoryGenerator(Node):

    def __init__(self):
        super().__init__('painting_trajectory_generator')

        self.publisher = self.create_publisher(
            PoseArray,
            '/painting/trajectory',
            10
        )

        self.timer = self.create_timer(2.0, self.publish_paths)

        self.get_logger().info(
            'Automotive painting trajectory generator started'
        )

    def generate_surface_path(
        self, x_start, x_end,
        y_start, y_end,
        z, rows=4
    ):

        poses = []

        for row in range(rows):

            y = y_start + (
                (y_end - y_start) * row / (rows - 1)
            )

            if row % 2 == 0:
                x_values = [
                    x_start + (x_end - x_start) * i / 30
                    for i in range(31)
                ]
            else:
                x_values = [
                    x_end - (x_end - x_start) * i / 30
                    for i in range(31)
                ]

            for x in x_values:

                pose = Pose()

                pose.position.x = x
                pose.position.y = y
                pose.position.z = z

                # Tool orientation placeholder:
                # nozzle direction is represented along local Z.
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0

                poses.append(pose)

        return poses

    def publish_paths(self):

        msg = PoseArray()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        # Approximate illustrative paths around the vehicle.
        # Coordinates must be calibrated against the actual car mesh.

        surfaces = [

            # Left side
            self.generate_surface_path(
                -1.5, 1.5,
                1.05, 1.05,
                0.85
            ),

            # Right side
            self.generate_surface_path(
                -1.5, 1.5,
                -1.05, -1.05,
                0.85
            ),

            # Roof
            self.generate_surface_path(
                -1.3, 1.3,
                -0.65, 0.65,
                1.75
            ),

            # Bonnet
            self.generate_surface_path(
                -2.0, -0.9,
                -0.65, 0.65,
                1.05
            ),

            # Boot
            self.generate_surface_path(
                0.9, 2.0,
                -0.65, 0.65,
                1.05
            )

        ]

        for surface in surfaces:
            msg.poses.extend(surface)

        self.publisher.publish(msg)

        self.get_logger().info(
            f'Published {len(msg.poses)} painting waypoints'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PaintingTrajectoryGenerator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
