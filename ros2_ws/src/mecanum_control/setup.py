from setuptools import find_packages, setup

package_name = 'mecanum_control'

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
    maintainer='KamalaIssack',
    maintainer_email='isaackamala11@gmail.com',
    description='Mecanum drive kinematics and Nucleo serial bridge',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'mecanum_kinematics = mecanum_control.mecanum_kinematics:main',
        'mecanum_odometry = mecanum_control.mecanum_odometry:main',
        ],
    },
)
