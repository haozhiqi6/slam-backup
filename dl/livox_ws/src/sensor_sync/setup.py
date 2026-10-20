from setuptools import find_packages, setup

package_name = "sensor_sync"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sensor_sync.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="haozhiqi",
    maintainer_email="haozhiqi@todo.todo",
    description="STM32 硬件时间同步的 ROS 2 桥接",
    license="BSD",
    entry_points={
        "console_scripts": [
            "epson_imu_node = sensor_sync.epson_imu_node:main",
            "check_align = sensor_sync.check_align:main",
        ],
    },
)
