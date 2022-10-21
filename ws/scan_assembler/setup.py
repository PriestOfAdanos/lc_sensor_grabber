import os
from glob import glob

from setuptools import setup

package_name = 'scan_assembler'

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
    description='Master node - manages all things necessary to collect data',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_assembler_node = scan_assembler.scan_assembler_node:main'
        ],
    },
)
