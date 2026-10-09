from glob import glob

from setuptools import find_packages, setup

package_name = 'flatbed_falcon'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Flatbed Falcon',
    maintainer_email='max.eidet@gmail.com',
    description='UAV precision landing on a moving UGV (pickup truck)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'truck_driver = flatbed_falcon.truck_driver:main',
            'target_tracker = flatbed_falcon.target_tracker:main',
            'landing_controller = flatbed_falcon.landing_controller:main',
            'logger = flatbed_falcon.logger:main',
        ],
    },
)
