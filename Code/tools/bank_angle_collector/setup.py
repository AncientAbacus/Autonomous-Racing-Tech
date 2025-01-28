from setuptools import setup
import os
from glob import glob

package_name = "bank_angle_collector"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Haoru Xue",
    maintainer_email="haorux@andrew.cmu.edu",
    description="collect bank angle and put in TTL",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bank_angle_collector_node = bank_angle_collector.bank_angle_collector_node:main"
        ]
    },
)
