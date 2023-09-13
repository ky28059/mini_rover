from setuptools import find_packages, setup

package_name = 'mini_rover'

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
    maintainer='Kevin Yu',
    maintainer_email='yu1271@purdue.edu',
    description='Boiler Robotics ROS mini rover challenge.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = mini_rover.minimal_publisher:main',
            'minimal_subscriber = mini_rover.minimal_subscriber:main',
        ],
    },
)
