from setuptools import setup


package_name = "bringup"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/coordinator.yaml"]),
        (f"share/{package_name}/launch", ["launch/demo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Student",
    maintainer_email="student@example.com",
    description="Top-level system bringup and workflow coordination.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "coordinator_node = bringup.coordinator_node:main",
            "startup_homing_node = bringup.startup_homing_node:main",
            "startup_pose_node = bringup.startup_pose_node:main",
        ]
    },
)
