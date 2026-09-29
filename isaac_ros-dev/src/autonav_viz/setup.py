from glob import glob

from setuptools import find_packages, setup

package_name = 'autonav_viz'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='barrettlarson',
    maintainer_email='cbarrettl.jr@gmail.com',
    description='RViz visualization suite for the AutoNav CV pipeline',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_viz_stub = autonav_viz.perception_viz_stub:main',
        ],
    },
)
