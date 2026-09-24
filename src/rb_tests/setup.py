from setuptools import setup
import os
from glob import glob

package_name = "rb_tests"
subpackages = [
    package_name,
    package_name + ".fakes",
    package_name + ".checks",
]

setup(
    name=package_name,
    version="0.1.0",
    packages=subpackages,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    # 声明它，colcon test 才会用 pytest 跑 test/ 目录
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="BUPT RobotTeam",
    maintainer_email="team@bupt.edu.cn",
    description="RoboCup basketball robot integration tests and fixtures",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # 测试夹具（供 pytest 与 launch 文件共用）
            "fake_board = rb_tests.fakes.fake_board:main",
            "fake_detections = rb_tests.fakes.fake_detections:main",
            "make_test_image = rb_tests.fakes.make_test_image:main",
            "publish_test_image = rb_tests.fakes.publish_test_image:main",
            # 离线/硬件检查（人工可跑，也供 pytest 包装调用）
            "verify_offline = rb_tests.checks.verify_offline:main",
            "verify_goto_avoid = rb_tests.checks.verify_goto_avoid:main",
            "verify_lidar_replay = rb_tests.checks.verify_lidar_replay:main",
            "camera_check = rb_tests.checks.camera_check:main",
            "vision_rate_check = rb_tests.checks.vision_rate_check:main",
        ],
    },
)
