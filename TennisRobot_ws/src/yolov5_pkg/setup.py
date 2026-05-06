from setuptools import setup, find_packages
import os
from glob import glob
package_name = 'yolov5_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov5_detectV1 = yolov5_pkg.yolov5_detectV1:main',
            'yolov5_detectV2 = yolov5_pkg.yolov5_detectV2:main',
        ],
    },
)
