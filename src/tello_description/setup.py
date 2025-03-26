from setuptools import setup
import os
from glob import glob

package_name = 'tello_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install model files
        (os.path.join('share', package_name, 'models', 'tello'), 
            glob('models/tello/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@domain.com',
    description='Tello drone description package',
    license='Apache License 2.0',
)