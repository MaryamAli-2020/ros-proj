from setuptools import setup


package_name = "simulation"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/attachment.yaml"]),
        (f"share/{package_name}/launch", ["launch/gazebo.launch.py"]),
        (f"share/{package_name}/worlds", ["worlds/pick_scan_place.world"]),
        (
            f"share/{package_name}/models/work_table",
            ["models/work_table/model.config", "models/work_table/model.sdf"],
        ),
        (
            f"share/{package_name}/models/sorting_bin",
            ["models/sorting_bin/model.config", "models/sorting_bin/model.sdf"],
        ),
        (
            f"share/{package_name}/models/qr_object",
            ["models/qr_object/model.config", "models/qr_object/model.sdf"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Student",
    maintainer_email="student@example.com",
    description="Gazebo world and attachment helpers.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "grasp_attachment_node = simulation.grasp_attachment_node:main",
        ]
    },
)

