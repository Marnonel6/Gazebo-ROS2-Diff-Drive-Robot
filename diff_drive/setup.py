from setuptools import setup

package_name = 'diff_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                    'launch/ddrive_rviz.launch.py',
                                    'launch/ddrive.launch.py',
                                    'urdf/ddrive.urdf.xacro',
                                    'urdf/ddrive.gazebo.xacro',
                                    'config/ddrive_urdf.rviz',
                                    'config/ddrive.yaml',
                                    'worlds/ddrive.world']),
                ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marno Nel',
    maintainer_email='marnonel2023@u.northwestern.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['flip=diff_drive.flip:flip_node_entry'
        ],
    },
)
