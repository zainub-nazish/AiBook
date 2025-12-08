"""Setup script for vla_control ROS 2 package."""

from setuptools import find_packages, setup

package_name = 'vla_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Vision-Language-Action control package for voice-controlled humanoid robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_command_node = vla_control.voice_command_node:main',
            'llm_planner_node = vla_control.llm_planner_node:main',
            'plan_executor_node = vla_control.plan_executor_node:main',
        ],
    },
)
