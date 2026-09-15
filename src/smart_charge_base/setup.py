from glob import glob

from setuptools import setup

package_name = "smart_charge_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Robot description for smart_charge_robot.",
    license="Apache-2.0",
)
