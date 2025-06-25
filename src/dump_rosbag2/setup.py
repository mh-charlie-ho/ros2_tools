import os
from glob import glob

from setuptools import find_packages, setup

package_name = "dump_rosbag2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dump_to_pickle = dump_rosbag2.dump_to_pickle:main",
            "dump_pc2_to_npy = dump_rosbag2.dump_pc2_to_npy:main",
            "dump_egostate_to_npy = dump_rosbag2.dump_egostate_to_npy:main",
            "convert_npy_to_feather = dump_rosbag2.convert_npy_to_feather:main",
            "batch_convert_pc2_pipeline = dump_rosbag2.batch_convert_pc2_pipeline:main",
            "calibration = dump_rosbag2.calibration:main",
        ],
    },
)
