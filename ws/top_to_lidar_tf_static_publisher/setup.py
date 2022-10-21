from setuptools import setup

package_name = 'top_to_lidar_tf_static_publisher'

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
    maintainer='lc',
    maintainer_email='pawelhabrzyk@gmail.com',
    description='Publish transformation between base and lidar',
    license='PROPRIETARY',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'top_to_lidar_tf_static_publisher_node = top_to_lidar_tf_static_publisher.top_to_lidar_tf_static_publisher_node:main'
        ],
    },
)
