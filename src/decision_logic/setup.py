from setuptools import setup


package_name = "decision_logic"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/bin_map.yaml"]),
        (f"share/{package_name}/launch", ["launch/decision.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Student",
    maintainer_email="student@example.com",
    description="Decision logic nodes for QR-based bin routing.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "decision_node = decision_logic.decision_node:main",
        ]
    },
)

