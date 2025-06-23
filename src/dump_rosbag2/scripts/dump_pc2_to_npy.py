import os
import argparse
import rosbag2_py
import numpy as np
from tqdm import tqdm
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class DumpPC2ToNpy:
    def __init__(self, topics: list, target_dir: str) -> None:
        self.topics = topics
        self.target_dir = target_dir

        self.reader = rosbag2_py.SequentialReader()

    def get_bag_info_without_conversion(self, file_path, file_type):
        storage_options = rosbag2_py.StorageOptions(
            uri=file_path,
            storage_id=file_type,
        )
        converter_options = rosbag2_py.ConverterOptions("", "")
        self.reader.open(storage_options, converter_options)
        self.topic_types = {
            t.name: t.type for t in self.reader.get_all_topics_and_types()
        }

    def dump(self):
        if not hasattr(self, "topic_types") or not self.topic_types:
            raise RuntimeError("Missing or empty topic type metadata.")

        os.makedirs(self.target_dir, exist_ok=True)
        with tqdm(desc="Dumping PointCloud2 to .npy", unit=" msg", leave=True) as pbar:
            while self.reader.has_next():
                topic, data, timestamp = self.reader.read_next()
                if topic not in self.topics:
                    continue

                msg_type = self.topic_types[topic]
                if msg_type != "sensor_msgs/msg/PointCloud2":
                    continue

                topic_id = self.topics.index(topic)

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
        "--topics",
        nargs="+",
        required=True,
        help="List of PointCloud2 topics to dump (example: '/top_lidar_points')",
    )
    parser.add_argument(
        "--target-dir",
        default="/pc2_npy_cache",
        help="Directory to store .npy files (default: '/pc2_npy_cache')",
    )

    args = parser.parse_args()

    dumper = DumpPC2ToNpy(args.topics, args.target_dir)
    dumper.get_bag_info_without_conversion(args.bag_path, args.bag_type)
    dumper.dump()
