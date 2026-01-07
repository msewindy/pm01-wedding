from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wedding_interaction'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='Wedding interaction robot FSM for EngineAI PM01',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wedding_fsm_node = wedding_interaction.nodes.wedding_fsm_node:main',
            'fsm_test = wedding_interaction.nodes.fsm_test_node:main',
            'motion_adapter = wedding_interaction.nodes.motion_adapter_node:main',
            'speech_player = wedding_interaction.nodes.speech_player_node:main',
            'perception_node = wedding_interaction.nodes.perception_node:main',
            'perception_visualizer = wedding_interaction.nodes.perception_visualizer_node:main',
        ],
    },
)

