import os

from setuptools import find_packages, setup

package_name = "robot_sdk_tests"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            ["launch/motion_chain.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robot SDK",
    maintainer_email="3464428214@qq.com",
    description="跨包验收探针",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 指向 cli 而不是 main —— console_scripts 不看返回值，退出码得自己转。
            "probe_motion = robot_sdk_tests.probe_motion:cli",
            "probe_wrap = robot_sdk_tests.probe_wrap:cli",
            "probe_avoid = robot_sdk_tests.probe_avoid:cli",
            "probe_accel = robot_sdk_tests.probe_accel:cli",
        ],
    },
)
