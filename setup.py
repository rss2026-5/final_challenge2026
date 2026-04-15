import glob
import os
from setuptools import find_packages
from setuptools import setup

package_name = 'final_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch', 'sim'),
         glob.glob(os.path.join('launch', 'sim', '*launch.*'))),
        (os.path.join('share', package_name, 'launch', 'real'),
         glob.glob(os.path.join('launch', 'real', '*launch.*'))),
        (os.path.join('share', package_name, 'config', 'sim'),
         glob.glob('config/sim/*.yaml')),
        (os.path.join('share', package_name, 'config', 'real'),
         glob.glob('config/real/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
         glob.glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohandas',
    maintainer_email='rohan.das.248@gmail.com',
    description='MIT RSS Final Challenge 2026',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Part A: The Great Snail Race
            'lane_detector = final_challenge.lane_detector:main',
            'lane_follower = final_challenge.lane_follower:main',
            'race_safety_controller = final_challenge.race_safety_controller:main',
            # Part B: Mrs. Puff's Boating School
            'overall_controller = final_challenge.overall_controller:main',
            'object_detector = final_challenge.object_detector:main',
            'traffic_light_detector = final_challenge.traffic_light_detector:main',
            # Reused from path_planning
            'safety_controller = final_challenge.safety_controller:main',
            'trajectory_planner = final_challenge.trajectory_planner:main',
            'trajectory_follower = final_challenge.trajectory_follower:main',
            'rrt_planner = final_challenge.rrt_planner:main',
        ],
    },
)
