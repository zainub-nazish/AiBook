"""Setup file for isaac_perception package."""
from setuptools import find_packages, setup

package_name = 'isaac_perception'

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
    description='Isaac ROS perception integration examples for humanoid robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add node entry points here as they are created
            # 'perception_demo = isaac_perception.perception_demo:main',
        ],
    },
)
