from setuptools import setup
from glob import glob

package_name = 'ros_x_habitat'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch/", glob('launch/*launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='remon',
    maintainer_email='RemonComputer@gmail.com',
    description='A bridge interface between Habitat Reinforcement learning'
                ' simulator and ROS 2',
    license='CC-BY-4.0',
    tests_require=['pytest'],
    package_data={'': ['*.py', '*/*.py', '*/*/*.py']},
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'joy_to_habitat = ros_x_habitat.nodes.joy_controller:main',
            'habitat_env = ros_x_habitat.nodes.habitat_env_node:main',
            'ptgoal_with_gps_compass_dummy_subscriber = ros_x_habitat.nodes.'
                'dummy_ptgoal_with_gps_compass_subscriber:main',
            'gazebo_to_habitat_agent = ros_x_habitat.nodes'
                '.gazebo_to_habitat_agent:main',
            'habitat_agent_to_gazebo = ros_x_habitat.nodes'
                '.habitat_agent_to_gazebo:main',
            'habitat_agent = ros_x_habitat.nodes.habitat_agent_node:main',
        ],
    },
)
