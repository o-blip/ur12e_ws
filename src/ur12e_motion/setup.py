from setuptools import find_packages, setup

package_name = 'ur12e_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/trac_ik_kinematics.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/ur_trac_ik_moveit.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='piero',
    maintainer_email='piero@todo.todo',
    description='UR12e winding experiments with TRAC-IK and reference prism geometry.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_client = ur12e_motion.ik_client:main',
            'winding_visualizer = ur12e_motion.winding_visualizer:main',
        ],
    },
)
