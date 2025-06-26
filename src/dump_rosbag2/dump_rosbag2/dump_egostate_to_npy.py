import os
import argparse
import rosbag2_py
import numpy as np
from tqdm import tqdm
from rclpy.serialization import deserialize_message
from custom_interfaces.msg import CarMotionState
import pyarrow as pa
import pyarrow.feather as feather


def euler_to_quaternion(roll, pitch, yaw):
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


class DumpEgoStateTofeather:
    def __init__(self, bag_dir: str, bag_type: str, topic: str, target_dir: str) -> None:
        self.bag_dir = bag_dir
        self.bag_type = bag_type
        self.topic = topic
        self.target_dir = target_dir
        os.makedirs(self.target_dir, exist_ok=True)

    def get_bag_files(self):
        return [
            os.path.join(self.bag_dir, f)
            for f in os.listdir(self.bag_dir)
            if f.endswith(f".{self.bag_type}")
        ]

    def read_topic_from_bag(self, file_path: str):
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=file_path, storage_id=self.bag_type)
        converter_options = rosbag2_py.ConverterOptions("", "")
        reader.open(storage_options, converter_options)

        topic_data = []
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == self.topic:
                msg = deserialize_message(data, CarMotionState)
                roll, pitch, yaw = (
                    msg.orientation_rpy.x,
                    msg.orientation_rpy.y,
                    msg.orientation_rpy.z,
                )
                qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
                topic_data.append(
                    [
                        timestamp,
                        qw,
                        qx,
                        qy,
                        qz,
                        msg.position.x,
                        msg.position.y,
                        msg.position.z,
                    ]
                )
        return topic_data

    # def test(self):
    #     bag_dir = "/media/charlie/ExtData2GB/bag/ros2/20250521-itri-p4-all-lidar-raw/rosbag2_2025_05_21-18_55_10"
    #     bag_type = "mcap"
    #     bag_files = self.get_bag_files(bag_dir, bag_type)

    #     bag_count = 0
    #     for bag_file in bag_files:
    #         self.read_topic_from_bag(bag_file, bag_type)
    #         bag_count += 1

    #     print("bag_count", bag_count)

    def save_to_feather(self, lists):
        np_data = np.array(lists, dtype=np.float64)
        timestamp_ns = np_data[:, 0].astype(np.int64)
        table = pa.table(
            {
                "timestamp_ns": timestamp_ns,
                "qw": np_data[:, 1],
                "qx": np_data[:, 2],
                "qy": np_data[:, 3],
                "qz": np_data[:, 4],
                "tx_m": np_data[:, 5],
                "ty_m": np_data[:, 6],
                "tz_m": np_data[:, 7],
            }
        )
        output_path = os.path.join(self.target_dir, "ego_state.feather")
        feather.write_feather(table, output_path)
        print("save to feather!")

    def run(self):
        all_data = []
        bag_files = self.get_bag_files()

        for bag_file in bag_files:
            data = self.read_topic_from_bag(bag_file)
            all_data.extend(data)
            print(f"Processing bag {bag_file}, messages num {len(data)}")

        if not all_data:
            return

        self.save_to_feather(all_data)


def main():
    parser = argparse.ArgumentParser(description="Dump CarMotionState to .feather from ROS2 bags")

    parser.add_argument(
        "--bag-dir",
        required=True,
        help="Path to the rosbag directory (contains multiple .mcap or other files)",
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
        default="./ego_feather_cache",
        help="Directory to store .feather file (default: './ego_feather_cache')",
    )

    args = parser.parse_args()

    dumper = DumpEgoStateTofeather(args.bag_dir, args.bag_type, args.topic, args.target_dir)
    dumper.run()
    

if __name__ == "__main__":
    main()
