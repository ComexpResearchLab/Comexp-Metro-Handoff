from glob import glob

from setuptools import setup

package_name = 'metro_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_data={package_name: ['sensor_hesai128.npz']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='morfjesus',
    description='Foreign-object detection ahead of a metro train from a Hesai OT128 lidar',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'metro_detector = metro_detector.node:main',
        ],
    },
)
