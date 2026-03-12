from setuptools import find_packages, setup

package_name = 'go2_my_nodes_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/hole_detection_clean.launch.py',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/hole_detection.rviz',
        ]),
        ('share/' + package_name + '/config', [  # Neu: Config-Verzeichnis
            'config/cardboard_detection_params.yaml',
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Helena Bongartz',
    maintainer_email='helenabongartz@users.noreply.github.com',
    description='Python ROS 2 nodes for Unitree GO2 entrance detection and motion control.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cloud_to_entrance_node = go2_my_nodes_py.cloud_to_entrance_node:main',
            'entrance_decision_node = go2_my_nodes_py.entrance_decision_node:main',
            'action_node = go2_my_nodes_py.action_node:main',
            'hole_detection_clean = go2_my_nodes_py.hole_detection_clean:main',
            'hole_detection_ransac = go2_my_nodes_py.hole_detection_ransac:main',
        ],
    },


)
