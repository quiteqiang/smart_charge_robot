from setuptools import find_packages, setup
from glob import glob

package_name = 'smart_charge_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smart_charge_robot',
    maintainer_email='dev@example.com',
    description='Top-level bringup for smart_charge_robot',
    license='Apache-2.0',
)
