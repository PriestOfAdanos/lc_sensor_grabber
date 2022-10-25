import os
from glob import glob

from setuptools import setup

package_name = 'engine_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paweł Habrzyk',
    maintainer_email='pawelhabrzyk@gmail.com',
    description='ros package to send current to correct pins to control stepper motor',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'engine_controller_node = engine_controller.engine_controller_node:main'
        ],
    },
)
