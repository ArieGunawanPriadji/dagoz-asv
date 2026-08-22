from setuptools import find_packages, setup

package_name = 'ball_detector_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if False else []),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bertrand',
    maintainer_email='bertrand@todo.todo',
    description='YOLO Buoy and Ball Detector for ASV KKI 2026',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = ball_detector_pkg.detector_node:main',
        ],
    },
)