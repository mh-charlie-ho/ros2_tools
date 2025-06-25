import os

# import pandas as pd
import numpy as np
import pyarrow as pa
import pyarrow.feather as feather


class Calibration:
    def __init__(self) -> None:
        pass

    def alignment(self, dir: str, file: str):
        table = feather.read_table(file)
        frame_infos = table.to_pydict()

        file_timestamps = []
        for f in os.listdir(dir):
            stem = os.path.splitext(f)[0]
            if stem.isdigit():
                file_timestamps.append(int(stem))

        if not file_timestamps:
            raise ValueError("No valid timestamp filenames found in directory.")

        file_timestamps = np.array(sorted(file_timestamps))

        column_names = list(frame_infos.keys())

        best_matches = {}
        num_rows = len(frame_infos["timestamp_ns"])
        for i in range(num_rows):
            original_ts = frame_infos["timestamp_ns"][i]
            idx = np.abs(file_timestamps - original_ts).argmin()
            aligned_ts = file_timestamps[idx]
            error = abs(original_ts - aligned_ts)

            if aligned_ts not in best_matches or error < best_matches[aligned_ts][0]:
                row_dict = {col: frame_infos[col][i] for col in column_names}
                row_dict["timestamp_ns"] = aligned_ts  # 替換為對齊值
                best_matches[aligned_ts] = (error, row_dict)

        output_dict = {col: [] for col in column_names}
        for _, row in best_matches.values():
            for col in column_names:
                output_dict[col].append(row[col])

        result_table = pa.table(output_dict)
        output_path = os.path.splitext(file)[0] + "_aligned_unique.feather"
        feather.write_feather(result_table, output_path)
        print(f"✅ Saved result with {result_table.num_rows} rows to {output_path}")


if __name__ == "__main__":
    calibration = Calibration()
    calibration.alignment(
        "/data/charlie/BagMonster/Dataset/bag_cache/rosbag2_2025_05_21-18_55_10_0/sensors/lidar",
        "/data/charlie/BagMonster/Dataset/bag_cache/rosbag2_2025_05_21-18_55_10_0/city_SE3_egovehicle_origin.feather",
    )
