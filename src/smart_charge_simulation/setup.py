from glob import glob

from setuptools import setup

package_name = "smart_charge_simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Minimal Nav2-compatible simulator (mining truck kinematics + ray-cast laser).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mining_truck_sim_node = smart_charge_simulation.mining_truck_sim_node:main",
        ],
    },
)
