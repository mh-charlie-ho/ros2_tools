import os
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
# from launch_ros.actions import Node


def get_config_file():
    config = os.path.join(
        get_package_share_directory("dump_rosbag2"),
        "config",
        "params.yaml",
    )
    return config


def load_yaml_file(path):
    yaml_path = Path(path).with_suffix(".yaml")
    with yaml_path.open("r") as file:
        params = yaml.safe_load(file)
    return params


def generate_launch_description():
    config = get_config_file()
    params = load_yaml_file(config)

    bag_dir = params["batch_convert_pc2_pipeline"]["bag_dir"]
    bag_type = params["batch_convert_pc2_pipeline"]["bag_type"]
    topics = params["batch_convert_pc2_pipeline"]["topics"]
    target_dir = params["batch_convert_pc2_pipeline"]["target_dir"]

    topics = (",").join(topics)

    batch_convert_pc2_pipeline = ExecuteProcess(
            cmd=[
                "ros2", "run", "dump_rosbag2", "batch_convert_pc2_pipeline",
                "--bag-dir", bag_dir,
                "--bag-type", bag_type,
                "--topics", topics,
                "--target-dir", target_dir,
            ],
            output="screen"
        )

    return LaunchDescription(
        [
            batch_convert_pc2_pipeline,
        ]
    )
