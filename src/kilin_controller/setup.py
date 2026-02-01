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
        ],
    },
)
