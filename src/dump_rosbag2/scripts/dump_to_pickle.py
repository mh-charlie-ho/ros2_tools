import os
import pickle
import argparse
import rosbag2_py
from tqdm import tqdm


class DumpToPkl:
    def __init__(self, topics: list, target_dir: str) -> None:
        self.topics = topics
        self.target_dir = target_dir

        self.reader = rosbag2_py.SequentialReader()
        self.topic_metadata = []

    def get_bag_info_without_conversion(self, file_path, file_type):
        storage_options = rosbag2_py.StorageOptions(
            uri=file_path,
            storage_id=file_type,
        )
        converter_options = rosbag2_py.ConverterOptions("", "")
        self.reader.open(storage_options, converter_options)
        self.topic_metadata = self.reader.get_all_topics_and_types()
        # info = rosbag2_py.Info()
        # metadata = info.read_metadata(file_path, file_type)

    def dump(self):
        if not self.topic_metadata:
            raise RuntimeError("Missing or empty topic metadata.")

        os.makedirs(self.target_dir, exist_ok=True)
        with tqdm(desc="Dumping bag to .pkl", unit=" msg", leave=True) as pbar:
            while self.reader.has_next():
                topic, data, timestamp = self.reader.read_next()
                if topic not in self.topics:
                    continue

                record = {
                    "topic": topic,
                    "timestamp": timestamp,
                    "data": data,
                }

                topic_id = self.topics.index(topic)
                with open(
                    f"{self.target_dir}/record_{topic_id}_{timestamp}.pkl", "wb"
                ) as f:
                    pickle.dump(record, f)
                pbar.update(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dump bag to .pkl")

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
        help="List of topics to include (example: '/top_lidar_points')",
    )
    parser.add_argument(
        "--target-dir",
        default="/bag_cache",
        help="Directory to store .pkl files (default: '/bag_cache')",
    )

    args = parser.parse_args()

    dumper = DumpToPkl(args.topics, args.target_dir)
    dumper.get_bag_info_without_conversion(args.bag_path, args.bag_type)
    dumper.dump()
