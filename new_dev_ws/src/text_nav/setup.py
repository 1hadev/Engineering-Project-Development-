from setuptools import find_packages, setup

package_name = 'text_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/text_nav_launch.py']),  # This line includes the launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='kharelshadev@gmail.com',
    description='ROS 2 Python package for text-based navigation',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_nav_node = text_nav.text_nav_node:main',  # This points to the main function in text_nav.py
        ],
    },
)


