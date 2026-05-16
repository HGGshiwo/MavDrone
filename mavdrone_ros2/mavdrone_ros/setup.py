from glob import glob
from setuptools import find_packages, setup

package_name = "mavdrone_ros"


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*launch.[pxy][yma]*")),
        ("share/" + package_name + "/config", glob("config/*.y?ml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rick van Essen",
    maintainer_email="rick.vanessen@wur.nl",
    description="ROS interface for mavdrone",
    license="AGPL-3.0-or-later",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mavdrone = mavdrone_ros.mavdrone_node:main",
            "local_grid_broadcaster = mavdrone_ros.local_grid_broadcaster_node:main",
            "goal_pose_server = mavdrone_ros.goal_pose_server_node:main",
            "test_fly_to_location = mavdrone_ros.test_fly_to_location:main",
            "test_land = mavdrone_ros.test_land:main",
            "test_takeoff = mavdrone_ros.test_takeoff:main",
        ],
    },
)
