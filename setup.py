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
            'talker = mini_rover.publisher_member_function:main',
            'listener = mini_rover.subscriber_member_function:main',
        ],
    },
)
