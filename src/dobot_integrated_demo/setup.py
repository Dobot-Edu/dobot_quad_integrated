from glob import glob
import os

from setuptools import find_packages, setup

package_name = "dobot_integrated_demo"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
        (os.path.join("share", package_name, "wavs"), glob(os.path.join("..", "..", "wavs", "*.wav"))),
    ],
    install_requires=["setuptools", "requests", "pyyaml", "numpy"],
    zip_safe=True,
    maintainer="Dobot Demo Team",
    maintainer_email="dev@dobot.cc",
    description="Integrated voice, safety guard, and balance control demo for Dobot quadruped robots.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "voice_control_node = dobot_integrated_demo.voice.voice_control_node:main",
            "depth_bridge_node = dobot_integrated_demo.perception.depth_bridge_node:main",
            "imu_reader_node = dobot_integrated_demo.balance.imu_reader_node:main",
            "balance_compensator_node = dobot_integrated_demo.balance.balance_compensator_node:main",
            "attitude_plot_recorder_node = dobot_integrated_demo.balance.attitude_plot_recorder_node:main",
            "balance_monitor_node = dobot_integrated_demo.balance.demo_monitor_node:main",
            "motion_gateway_node = dobot_integrated_demo.core.motion_gateway_node:main",
            "safety_arbitrator_node = dobot_integrated_demo.core.safety_arbitrator_node:main",
            "integrated_state_manager_node = dobot_integrated_demo.core.integrated_state_manager_node:main",
        ],
    },
)
