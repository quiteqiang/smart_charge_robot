from setuptools import find_packages, setup

package_name = 'smart_charge_docking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/docking.launch.py']),
        ('share/' + package_name + '/config', ['config/dock_controller.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smart_charge_robot',
    maintainer_email='dev@example.com',
    description='Precise low-speed docking controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dock_controller_node = smart_charge_docking.dock_controller_node:main',
        ],
    },
)
