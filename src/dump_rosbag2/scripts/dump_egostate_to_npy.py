import os
import argparse
import rosbag2_py
import numpy as np
from tqdm import tqdm
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from car_motion_state.msg import CarMotionState


class DumpPC2ToNpy:
    def __init__(self, topic: list, target_dir: str) -> None:
        self.topic = topic
        self.target_dir = target_dir

        self.reader = rosbag2_py.SequentialReader()

    def get_bag_info_without_conversion(self, file_path, file_type):
        storage_options = rosbag2_py.StorageOptions(
            uri=file_path,
            storage_id=file_type,
        )
        converter_options = rosbag2_py.ConverterOptions("", "")
        self.reader.open(storage_options, converter_options)

    def dump(self):
        os.makedirs(self.target_dir, exist_ok=True)
        with tqdm(desc="Dumping ego state to .npy", unit=" msg", leave=True) as pbar:
            while self.reader.has_next():
                topic, data, timestamp = self.reader.read_next()
                if topic != self.topic:
                    continue

                topic_id = self.topic.index(topic)

                msg = deserialize_message(data, PointCloud2)
                points = [
                    [x, y, z, intensity]
                    for x, y, z, intensity in pc2.read_points(
                        msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
                    )
                ]
                np_points = np.array(points, dtype=np.float32)

                filename = f"{self.target_dir}/pc2_{topic_id}_{timestamp}.npy"
                np.save(filename, np_points)
                pbar.update(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Dump PointCloud2 data from ROS2 bag to .npy"
    )

    parser.add_argument(
        "--bag-path",
        required=True,
        help="Path to the rosbag (example: '/path/to/bag.mcap', etc.)",
    )
    parser.add_argument(
        "--bag-type",
        required=True,
        help="Storage type of the rosbag (example: 'mcap')",
    )
    parser.add_argument(
        "--topic",
        required=True,
        help="Car state topic to dump (example: '/localization/car_motion_state')",
    )
    parser.add_argument(
        "--target-dir",
        default="/ego_npy_cache",
        help="Directory to store .npy files (default: '/ego_npy_cache')",
    )

    args = parser.parse_args()

    dumper = DumpPC2ToNpy(args.topic, args.target_dir)
    dumper.get_bag_info_without_conversion(args.bag_path, args.bag_type)
    dumper.dump()
