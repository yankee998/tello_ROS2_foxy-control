from setuptools import setup

package_name = 'tello_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@domain.com',
    description='Tello drone controller package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'controller = tello_controller.controller:main',
        ],
    },
)