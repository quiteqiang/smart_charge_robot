from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smart_charge_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.pgm')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smart_charge_robot',
    maintainer_email='dev@example.com',
    description='Map, AMCL and trimmed Nav2 launch',
    license='Apache-2.0',
)
