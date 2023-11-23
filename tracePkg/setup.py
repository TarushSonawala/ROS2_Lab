from setuptools import find_packages, setup

package_name = 'tracePkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'robot_publisher=tracePkg.pub:main',
            'robot_subscriber=tracePkg.sub:main',
            'crazy_turtle=tracePkg.turtle_mad:main',
            'meow_add=tracePkg.serv:main'
        ],
    },
)
