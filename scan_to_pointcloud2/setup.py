from setuptools import setup

package_name = 'scan_to_pointcloud2'

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
    maintainer='phabrzyk',
    maintainer_email='pawelhabrzyk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_to_pointcloud2_node = scan_to_pointcloud2.scan_to_pointcloud2_node:main'
        ],
    },
)
