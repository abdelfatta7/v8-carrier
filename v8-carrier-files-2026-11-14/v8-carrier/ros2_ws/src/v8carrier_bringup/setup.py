from glob import glob

from setuptools import setup

package_name = "v8carrier_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/web", ["web/index.html"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="owner",
    maintainer_email="owner@example.com",
    description="Serial bridge, sensor filters, battery rules and web UI for the V8 carrier robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "serial_bridge = v8carrier_bringup.serial_bridge:main",
            "scan_filter = v8carrier_bringup.scan_filter_node:main",
            "battery_rules = v8carrier_bringup.battery_rules_node:main",
            "web_ui = v8carrier_bringup.web_ui_node:main",
        ],
    },
)
