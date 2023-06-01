from setuptools import setup

package_name = 'pc_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpu-mirea',
    maintainer_email='example@mail.com',
    description='3D-object detection with point cloud',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect = pc_detection.pc_detection:main'
        ],
    },
)
