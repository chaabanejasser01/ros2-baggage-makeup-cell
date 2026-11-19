from setuptools import find_packages, setup

package_name = 'bag_cell_scene'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/manifest.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasser',
    maintainer_email='jasser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scene_publisher = bag_cell_scene.scene_publisher:main',
            'belt_controller = bag_cell_scene.belt_controller:main',
            'flight_router = bag_cell_scene.flight_router:main',
        ],
    },
)
