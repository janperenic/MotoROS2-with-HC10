from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("motoman_hc10dt", package_name="hc10_movit2v3").to_moveit_configs()
    # spremeni hc10_movit2v3 za simulacijo in hc10dt_moveit_config za robota
    return generate_demo_launch(moveit_config)
