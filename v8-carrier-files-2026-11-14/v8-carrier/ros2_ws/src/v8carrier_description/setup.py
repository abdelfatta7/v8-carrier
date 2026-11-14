from glob import glob

from setuptools import setup

package_name = "v8carrier_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="owner",
    maintainer_email="owner@example.com",
    description="URDF of the V8 carrier robot, generated from cad/params.yaml.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "generate_urdf = v8carrier_description.urdf_generator:main",
        ],
    },
)
