from setuptools import find_packages, setup

package_name = 'yolo_pub'

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
    maintainer='prem',
    maintainer_email='sukhadwala.p@northeastern.edu',
    description='ROS publisher for yolo-based object detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'talker = yolo_pub.yolo_prediction2:main'
        ],
    },
)
