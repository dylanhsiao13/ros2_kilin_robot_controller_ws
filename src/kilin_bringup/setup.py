from setuptools import setup

package_name = 'kilin_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rviz_bringup.launch.py']),
        ('share/' + package_name + '/launch', ['launch/IsaacSim_bringup.launch.py']),
        ('share/' + package_name + '/launch', ['launch/joint_state_plotter.py']),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='dylanhsiao13@gmail.com',
    description='Bringup package for Kilin robot',
    license='Apache-2.0',
)
