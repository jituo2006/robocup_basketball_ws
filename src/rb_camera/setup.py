from setuptools import setup
import os
from glob import glob

package_name = "rb_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    # 声明它，colcon test 才会用 pytest 跑 test/ 目录
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="BUPT RobotTeam",
    maintainer_email="team@bupt.edu.cn",
    description="RoboCup basketball robot UVC camera driver",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_node = rb_camera.camera_node:main",
        ],
    },
)
