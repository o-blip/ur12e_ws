from setuptools import find_packages, setup

package_name = 'dbar_tensegrity'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='piero',
    maintainer_email='piero@todo.todo',
    description='Minimal skeleton for planar D-bar tensegrity geometry.',
    license='Apache-2.0',
)
