from setuptools import find_packages, setup

package_name = 'smart_charge_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission.launch.py']),
        ('share/' + package_name + '/config', ['config/mission_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smart_charge_robot',
    maintainer_email='dev@example.com',
    description='Charging mission state machine',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'charge_mission_node = smart_charge_mission.charge_mission_node:main',
            'navigate_to_task = smart_charge_mission.navigate_to_task:main',
        ],
    },
)
