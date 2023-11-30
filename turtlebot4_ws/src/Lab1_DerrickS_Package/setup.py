from setuptools import find_packages, setup

package_name = 'Lab1_Derrick_Package'

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
    maintainer='neubert',
    maintainer_email='neubert@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot4_first_python_node = turtlebot4_python_tutorials.turtlebot4_first_python_node:main',
            'turtlebot4_wander_node = turtlebot4_python_tutorials.wander_node:main',
            'turtlebot4_undock_node = turtlebot4_python_tutorials.turtlebot4_undock_node:main'
        ],
    },
)