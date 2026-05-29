from setuptools import find_packages, setup


package_name = "igvc_competition_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config",
         ["config/igvc_competition_compact.yaml"]),
        ("share/" + package_name + "/launch",
         ["launch/igvc_competition.launch.py"]),
        ("share/" + package_name + "/worlds",
         ["worlds/igvc_competition_compact.sdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AutoNav",
    maintainer_email="autonav@vt.edu",
    description="Gazebo Fortress IGVC competition simulation harness.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "generate_igvc_world = igvc_competition_sim.generate_world:main",
            "igvc_camera_bridge = igvc_competition_sim.camera_bridge:main",
            "igvc_course_monitor = igvc_competition_sim.course_monitor:main",
            "igvc_mission_runner = igvc_competition_sim.mission_runner:main",
            "igvc_run_analyzer = igvc_competition_sim.run_analyzer:main",
            "igvc_sensor_harness = igvc_competition_sim.sensor_harness:main",
        ],
    },
)
