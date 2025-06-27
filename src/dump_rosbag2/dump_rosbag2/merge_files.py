import glob
import shutil
import argparse
from pathlib import Path


def merge_files(glob_pattern: str, target_dir: str, move: bool = False):
    matched_files = glob.glob(glob_pattern, recursive=True)
    if not matched_files:
        print(f"No files matched the pattern: {glob_pattern}")
        return

    target_path = Path(target_dir)
    target_path.mkdir(parents=True, exist_ok=True)

    for file_path in matched_files:
        src = Path(file_path)
        dst = target_path / src.name

        if dst.exists():
            print(f"Skipping {src.name}, already exists in target.")
            continue

        if move:
            shutil.move(str(src), str(dst))
            print(f"Moved {src} -> {dst}")
        else:
            shutil.copy2(str(src), str(dst))
            print(f"Copied {src} -> {dst}")


def main():
    parser = argparse.ArgumentParser(description="Merge matched files into one folder.")

    parser.add_argument(
        "--pattern",
        required=True,
        help="Glob pattern to match files, e.g. './a/b/*/*.jpg'",
    )
    parser.add_argument(
        "--target-dir", required=True, help="Target directory to collect matched files."
    )
    parser.add_argument("--move", action="store_true", help="Move instead of copy.")

    args = parser.parse_args()

    merge_files(args.pattern, args.target_dir, args.move)


if __name__ == "__main__":
    main()
