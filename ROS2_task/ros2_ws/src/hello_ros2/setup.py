from setuptools import find_packages, setup

package_name = 'hello_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kareem',
    maintainer_email='kareem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = hello_ros2.hello_node:main',
            'hello_subscriber = hello_ros2.hello_subscriber:main',
            'waypoint_navigator = hello_ros2.waypoint_navigator:main',
        ],
    },
)
