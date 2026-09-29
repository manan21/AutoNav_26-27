from glob import glob

from setuptools import setup

package_name = 'autonav_viz'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],   # config-only: installs launch/ and rviz/ as data files
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
    description='RViz config and launch for the AutoNav CV pipeline',
    license='BSD-3-Clause',
    tests_require=['pytest'],
)
