from setuptools import find_packages, setup

package_name = "robot_sdk_mcp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # mcp 钉 <2，与 Walle 同版本；v1/v2 对同步工具的处理相反。
    install_requires=["setuptools", "mcp>=1.0,<2"],
    zip_safe=True,
    maintainer="Robot SDK",
    maintainer_email="3464428214@qq.com",
    description="把 Robot SDK 的四类能力暴露为 MCP 工具",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "robot_sdk_mcp_server = robot_sdk_mcp.server:main",
        ],
    },
)
