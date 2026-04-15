from setuptools import setup
import os
from glob import glob

package_name = 'neroFly'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'worlds/meshes'), glob('worlds/meshes/*.dae')),
        (os.path.join('share', package_name, 'worlds/textures'), glob('worlds/textures/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brayan',
    description='Bebop simulation with reference arrow visualizer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simulator_node = neroFly.simulator_node:main',
            'visualizer_node = neroFly.visualizer_node:main',
            'pionner_node = neroFly.pionner_node:main',
        ],
    },
)