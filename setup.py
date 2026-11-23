from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'skytrace_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (
            os.path.join('share', package_name, 'meshes'),
            [f for f in glob('meshes/*') if os.path.isfile(f)],
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            [f for f in glob('urdf/*') if os.path.isfile(f)],
        ),
        (
            os.path.join('share', package_name, 'web'),
            [f for f in glob('web/*') if os.path.isfile(f)],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tanish Sharma',
    maintainer_email='tanish@todo.todo',
    description='SKYTRACE precision target detection and tracking simulation in RViz2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'skytrace_sim_node = skytrace_sim.skytrace_sim_node:main',
            'skytrace_aircraft_node = skytrace_sim.skytrace_aircraft_node:main',
            'skytrace_urdf_node = skytrace_sim.skytrace_urdf_node:main',
            'skytrace_opensky_node = skytrace_sim.skytrace_opensky_node:main',
        ],
    },
)
