from setuptools import setup
from glob import glob
import os

package_name = 'kilin_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhsiao',
    maintainer_email='dylanhsiao13@gmail.com',
    description='Kilin robot description package',
    license='Apache-2.0',
)
