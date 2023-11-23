from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),glob('launch/*')),
        (os.path.join('share',package_name),glob('urdf/*')),
        (os.path.join('share',package_name),glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trace_paradox',
    maintainer_email='trace_paradox@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control=lab_5.controller:main'
        ],
    },
)
