import os
import argparse
from tqdm import tqdm

from dump_rosbag2.dump_pc2_to_npy import DumpPC2ToNpy
from dump_rosbag2.convert_npy_to_feather import NpyToFeather


def run_pipeline(bag_path: str, bag_type: str, target_dir: str, topics: list):
    base_name = os.path.splitext(os.path.basename(bag_path))[0]

    npy_dir = os.path.join(target_dir, base_name, "npy")
    feather_dir = os.path.join(target_dir, base_name, "feather")
    os.makedirs(npy_dir, exist_ok=True)
    os.makedirs(feather_dir, exist_ok=True)

    dumper = DumpPC2ToNpy(topics, npy_dir)
    dumper.get_bag_info_without_conversion(bag_path, bag_type)
    dumper.dump()

    converter = NpyToFeather(npy_dir, feather_dir)
    converter.convert()


def main():
    parser = argparse.ArgumentParser(
        description="Batch convert ROS2 bags to .npy and .feather"
    )
    parser.add_argument(
        "--bag-dir",
        type=str,
        required=True,
        help="Directory containing ROS2 bag files (.mcap, .db3, etc.)",
    )
    parser.add_argument(
        "--bag-type",
        type=str,
        required=True,
        help="Storage type of the ROS2 bag (example: 'mcap')",
    )
    parser.add_argument(
        "--target-dir",
        type=str,
        default="/converted_output",
        help="Directory to store converted output (default: /converted_output)",
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        required=True,
        help="List of topics to extract (default: ['/top_lidar_points'])",
    )

    args = parser.parse_args()

    bag_files = [f for f in os.listdir(args.bag_dir) if f.endswith(f".{args.bag_type}")]

    for fname in tqdm(
        bag_files, total=len(bag_files), desc="Processing bags", unit=" bag"
    ):
        bag_path = os.path.join(args.bag_dir, fname)
        run_pipeline(bag_path, args.bag_type, args.target_dir, args.topics)


if __name__ == "__main__":
    main()
