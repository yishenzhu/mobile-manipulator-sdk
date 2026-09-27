import os

from setuptools import setup

package_name = "robot_sdk_tests"

setup(
    name=package_name,
    version="0.1.0",
    # 显式列出目录名。**不能用 find_packages()** —— 那个靠 `__init__.py` 认包，
    # 而这里没放（目录里只有几个脚本），于是普通安装（不带 --symlink-install）
    # 会只装入口脚本、不装模块，运行时报 ModuleNotFoundError。
    packages=[package_name],
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
    # 判定走 pytest（见 test/）。探针模块本身不带入口 —— 判定只有一条路，
    # 不用再维护"退出码"那一套。人要看某一条：
    #     pytest src/robot_sdk_tests/test/test_probes.py -s -k motion
    tests_require=["pytest"],
)
