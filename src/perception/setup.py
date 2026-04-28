from setuptools import setup


package_name = "perception"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/perception.yaml"]),
        (f"share/{package_name}/launch", ["launch/perception.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Student",
    maintainer_email="student@example.com",
    description="Perception bridge and QR demo stream tools.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "qr_decoder_bridge = perception.qr_decoder_bridge:main",
            "mock_qr_stream = perception.mock_qr_stream:main",
        ]
    },
)

