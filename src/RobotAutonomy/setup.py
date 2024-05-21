import os
from glob import glob
from setuptools import find_packages, setup

package_name = "my_turtlebot"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]


def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for path, directories, filenames in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join("share", package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=package_files(
        data_files, ["launch", "rviz", "urds", "worlds", "maps", "models"]
    ),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "turtle_circles = my_turtlebot.turtle_circles:main",
            "move_turtlebot = my_turtlebot.move_turtlebot:main",
            "lidar_sub = my_turtlebot.lidar_sub:main",
            "lidar_icp = my_turtlebot.lidar_icp:main",
            "lidar_kiss = my_turtlebot.lidar_kiss:main",
            "map_random = my_turtlebot.map_random:main",
            "map_lidar_static = my_turtlebot.map_lidar_static:main",
            "map_lidar = my_turtlebot.map_lidar:main",
            "particle_sub = my_turtlebot.particle_sub:main",
            "planner_prm = my_turtlebot.planner_prm:main",
            "explore_nbv = my_turtlebot.explore_nbv:main",
            "report_nbv = my_turtlebot.report_nbv:main",
        ],
    },
)
