"""This is a minimal example to read a ROS 2 bag file using the rosbag2_py library."""

import os
import rosbag2_py
import pickle

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

import sensor_msgs_py.point_cloud2 as pc2

import numpy as np

import json

# ---------------------------------------------------------- save to pickle
"""
reader = rosbag2_py.SequentialReader()

# 設定存儲選項
storage_options = rosbag2_py.StorageOptions(
    uri="/data/object_not_stably_tracked0.mcap",  # 不要加副檔名
    storage_id="mcap",  # 指定 mcap
)

# 不需要轉換器選項
# 如果需要轉換器選項，可以使用 rosbag2_py.ConverterOptions("converter_id", "serialization_format")
# 例如: rosbag2_py.ConverterOptions("cdr", "cdr")
# 這裡的 converter_id 和 serialization_format 可以根據需要進行調整
converter_options = rosbag2_py.ConverterOptions("", "")

# 開啟讀取器
reader.open(storage_options, converter_options)

# 列出 topic 與 type
topics = reader.get_all_topics_and_types()

print("==========================")
os.makedirs("bag_cache", exist_ok=True)
save_raw_data_enabled = True  # 是否儲存原始資料
# records = []  # 用於儲存讀取的資料
count = 0  # 計數器
if save_raw_data_enabled is True:
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        record = {
            "topic": topic,
            "timestamp": timestamp,
            "data": data,  # still serialized
        }

        with open(f"bag_cache/record_{count:06d}.pkl", "wb") as f:
            pickle.dump(record, f)
        count += 1
"""
# ---------------------------------------------------------- load from pickle

# convert point cloud data to numpy array (npy)
record = {}
with open("/bag_cache/record_103547.pkl", "rb") as f:
    record = pickle.load(f)

print(record.keys())
print(record["topic"])
print(record["timestamp"])
msg = deserialize_message(record["data"], PointCloud2)
print(msg.header)

points = []
for pt in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
    points.append([pt[0], pt[1], pt[2], pt[3]])


if not points:
    pass
else:
    print(f"Number of points: {len(points)}")
    print(f"First point: {points[0]}")
    print(f"Last point: {points[-1]}")

    arr = np.array(points, dtype=np.float32)

    # 儲存為 npy
    np.save("/bag_cache/cloud_test.npy", arr)

# ---------------------------------------------------------- load from npy
arr = np.load("/bag_cache/cloud_test.npy")
print(arr.shape)       # (N, 3)
print(arr[:5])

# ---------------------------------------------------------- save to npy with metadata

# meta = []

# meta.append({
#     "filename": f"cloud_{count:06d}.npy",
#     "timestamp": timestamp,
#     "frame_id": msg.header.frame_id,
#     "num_points": len(arr)
# })

# with open(os.path.join(output_dir, "meta.json"), "w") as f:
#     json.dump(meta, f, indent=2)
