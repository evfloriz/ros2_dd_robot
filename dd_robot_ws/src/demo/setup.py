from setuptools import find_packages, setup

package_name = 'demo'

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
    maintainer='evan',
    maintainer_email='ev.floriz@gmail.com',
    description='Quick demo showcasing camera input and motor control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_ball = demo.follow_ball:main'
        ],
    },
)
