from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "automotive_painting_cell",
            package_name="painting_cell_moveit_config"
        )
        .robot_description(
            file_path="config/painting_cell_rail.urdf"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config)
