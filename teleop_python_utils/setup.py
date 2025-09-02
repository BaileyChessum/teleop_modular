from setuptools import find_packages, setup

package_name = 'teleop_python_utils'

setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bailey Chessum',
    maintainer_email='bailey.chessum1@gmail.com',
    description='Python utility classes for working with teleop_modular.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
