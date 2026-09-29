from setuptools import find_packages, setup

package_name = 'create3_pkg'

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
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talkerDemo = create3_pkg.talkerDemo:main',
            'test_bluecolorlight = create3_pkg.test_bluecolorlight:main',
            'test_subproximity_sensor = create3_pkg.test_subproximity_sensor:main',
        ],
    },
)