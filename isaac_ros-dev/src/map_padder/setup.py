from setuptools import find_packages, setup

package_name = 'map_padder'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nfikes',
    maintainer_email='nfikes@vt.edu',
    description='Pads the SLAM map to a minimum size for GPS waypoint navigation',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_padder_node = map_padder.map_padder_node:main',
        ],
    },
)
