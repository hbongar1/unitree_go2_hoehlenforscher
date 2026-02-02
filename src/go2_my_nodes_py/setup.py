from setuptools import find_packages, setup

package_name = 'go2_my_nodes_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/hole_detection_voxel_grid.launch.py',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/hole_detection.rviz',
        ]),
        ('share/' + package_name + '/config', [  # Neu: Config-Verzeichnis
            'config/cardboard_detection_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cloud_to_entrance_node = go2_my_nodes_py.cloud_to_entrance_node:main',
            'entrance_decision_node = go2_my_nodes_py.entrance_decision_node:main',
            'navigation_action_server_node = go2_my_nodes_py.navigation_action_server_node:main',
            'hole_detection_voxel_grid = go2_my_nodes_py.hole_detection_voxel_grid:main',
             'hole_detection_clean = go2_my_nodes_py.hole_detection_clean:main',
        ],
    },


)
