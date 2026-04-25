from setuptools import find_packages, setup

package_name = 'blackbox_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/recorder_launch.py']),
        ('share/' + package_name + '/config', ['config/recorder_params.yaml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='Black Box Robotics',
    maintainer_email='support@blackrobotics.in',
    description='Black Box Robotics episode recorder for ROS 2 robots',
    license='MIT',
    entry_points={
        'console_scripts': [
            'episode_recorder = blackbox_recorder.episode_recorder:main',
            'rosbag_exporter = blackbox_recorder.rosbag_exporter:main',
            'task_event = blackbox_recorder.task_event_helper:main',
        ],
    },
)
