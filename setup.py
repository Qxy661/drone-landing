from setuptools import find_packages, setup

package_name = "drone_landing"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/landing_test.launch.py",
            "launch/landing_system.launch.py",
        ]),
        ("share/" + package_name + "/config", [
            "config/landing_params.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="dev@example.com",
    description="Autonomous precision visual landing system",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "landing_detector = drone_landing.landing_detector:main",
            "landing_controller = drone_landing.landing_controller:main",
            "mission_planner = drone_landing.mission_planner:main",
        ],
    },
)
