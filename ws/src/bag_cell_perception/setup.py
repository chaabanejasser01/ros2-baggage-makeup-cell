from setuptools import find_packages, setup

package_name = 'bag_cell_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasser',
    maintainer_email='jasser@todo.todo',
    description='Overhead depth camera: measures the bag in the pick zone',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bag_detector = bag_cell_perception.bag_detector:main',
        ],
    },
)
