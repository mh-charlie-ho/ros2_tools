import os
import argparse
import numpy as np
import pyarrow as pa
import pyarrow.feather as feather
# import pandas as pd
from tqdm import tqdm


class NpyToFeather:
    def __init__(self, npy_dir: str, feather_dir: str) -> None:
        self.npy_dir = npy_dir
        self.feather_dir = feather_dir
        os.makedirs(self.feather_dir, exist_ok=True)

    def convert(self):
        npy_files = sorted(
            [f for f in os.listdir(self.npy_dir) if f.endswith(".npy")]
        )

        if not npy_files:
            raise RuntimeError("No .npy files found in the input directory.")

        for fname in tqdm(npy_files, total=len(npy_files), desc="Converting .npy to .feather", unit=" file"):
            npy_path = os.path.join(self.npy_dir, fname)
            points = np.load(npy_path)  # expects shape (N, 4) hard-coding

            if points.ndim != 2 or points.shape[1] != 4:
                raise ValueError(f"{fname} is not in expected (N, 4) format.")

            # df = pd.DataFrame(points, columns=["x", "y", "z", "intensity"]) # hard-coding
            # feather_name = fname.replace(".npy", ".feather")
            # df.to_feather(os.path.join(self.feather_dir, feather_name))

            # 使用 pyarrow 保留 float16 (halffloat) 格式
            arrays = [
                pa.array(points[:, 0], type=pa.float16()),
                pa.array(points[:, 1], type=pa.float16()),
                pa.array(points[:, 2], type=pa.float16()),
                pa.array(points[:, 3], type=pa.float16())
            ]
            table = pa.Table.from_arrays(arrays, names=["x", "y", "z", "intensity"])
            feather_name = fname.replace(".npy", ".feather")
            feather_path = os.path.join(self.feather_dir, feather_name)
            feather.write_feather(table, feather_path, version=2)  # 使用 Feather V2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert .npy point cloud to .feather")

    parser.add_argument(
        "--npy-dir",
        required=True,
        help="Directory containing .npy files.",
    )
    parser.add_argument(
        "--feather-dir",
        default="/feather_output",
        help="Directory to store .feather files (default: '/feather_output').",
    )

    args = parser.parse_args()

    converter = NpyToFeather(args.npy_dir, args.feather_dir)
    converter.convert()
