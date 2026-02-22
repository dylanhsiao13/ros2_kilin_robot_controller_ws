from setuptools import find_packages, setup

package_name = 'kilin_controller'

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
    maintainer='dhsiao',
    maintainer_email='dylanhsiao13@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_commander=kilin_controller.joint_commander:main',
            'whole_body_controller=kilin_controller.whole_body_controller:main',
            'joint_state_plotter=kilin_controller.joint_state_plotter:main',
            'dummy_simulator=kilin_controller.dummy_simulator:main',
            'state_estimator=kilin_controller.state_estimator:main',
            'state_estimation_visualizer=kilin_controller.state_estimation_visualizer:main',
            'base_attitude_controller=kilin_controller.base_attitude_controller:main',
            'base_attitude_gui=kilin_controller.base_attitude_gui:main',
            'joint_trajectory_planner=kilin_controller.joint_trajectory_planner:main',
            'custom_controller=kilin_controller.custom_controller:main',
            'custom_controller_gui=kilin_controller.custom_controller_gui:main',
            'stair_climbing_gait_generator=kilin_controller.stair_climbing_gait_generator:main',
            
        ],
    },
)
