from setuptools import find_packages, setup

package_name = 'manipulator_viz_core'

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
    maintainer='Prem Gandhi',
    maintainer_email='phgandh2@ncsu.edu',
    description='custom visualization tool for visualizing the data that is received from the ROS2 network',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'run_viz = manipulator_viz_core.run_viz:main'
        ],
    },
)
