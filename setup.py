from setuptools import setup

package_name = "my_planner_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/maps", ["maps/first_try.yaml", "maps/first_try.pgm"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Henrik",
    maintainer_email="henrik@todo.todo",
    description="A* global planner + potential field local planner",
    license="MIT",
    entry_points={
        "console_scripts": [
            "planner_pf_node = my_planner_pkg.planner_pf_node:main",
            "pf_localization = my_planner_pkg.particle_filter_localization:main",
            "frontier_pf_explorer = my_planner_pkg.frontier_pf_explorer_node:main",
            "set_initial_pose = my_planner_pkg.set_initial_pose:main",
        ],
    },
)