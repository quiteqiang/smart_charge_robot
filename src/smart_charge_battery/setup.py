from setuptools import find_packages, setup

package_name = 'smart_charge_battery'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/battery_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/battery.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smart_charge_robot',
    maintainer_email='dev@example.com',
    description='Battery SOC simulator node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'battery_simulator_node = smart_charge_battery.battery_simulator_node:main',
        ],
    },
)
