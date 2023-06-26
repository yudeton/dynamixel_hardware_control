import os
from glob import glob
from setuptools import find_packages, setup
from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'traj_excel_10'), glob(os.path.join('traj_excel', '*.xlsx'))),
        (os.path.join('share', package_name, 'traj_excel_30'), glob(os.path.join('traj_excel', '*.xlsx'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.STL'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'traj_excel_30_v2'), glob(os.path.join('traj_excel', '*.xlsx'))),
        (os.path.join('share', package_name, 'traj_excel_10_v2'), glob(os.path.join('traj_excel', '*.xlsx'))),
        (os.path.join('share', package_name, 'complex_traj_30'), glob(os.path.join('traj_excel', '*.xlsx'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iclab',
    maintainer_email='iclab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'talker = my_package.publisher_member_function:main',
            'action_client = my_package.action_member_function:main',
            'module_robot = my_package.module_robot:main',
            'listener = my_package.subscriber_member_function:main',
        ],
    },
)
