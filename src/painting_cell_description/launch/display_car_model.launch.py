from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    urdf_file = PathJoinSubstitution([
        FindPackageShare('painting_cell_description'),
        'urdf',
        'car_model.urdf'
    ])

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )

    ])

