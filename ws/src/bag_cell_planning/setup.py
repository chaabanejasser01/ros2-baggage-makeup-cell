from setuptools import find_packages, setup

package_name = 'bag_cell_planning'

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
    description='Cart manager (slots) and metrics logger',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cart_manager = bag_cell_planning.cart_manager:main',
            'metrics_logger = bag_cell_planning.metrics_logger:main',
        ],
    },
)
