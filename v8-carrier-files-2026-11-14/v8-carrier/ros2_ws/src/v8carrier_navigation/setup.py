from glob import glob

from setuptools import setup

package_name = "v8carrier_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="owner",
    maintainer_email="owner@example.com",
    description="SLAM/Nav2 configuration, coverage planner and mission node for the V8 carrier robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "coverage = v8carrier_navigation.coverage_node:main",
            "mission = v8carrier_navigation.mission_node:main",
        ],
    },
)
