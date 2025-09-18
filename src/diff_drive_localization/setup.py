from setuptools import find_packages, setup

package_name = 'diff_drive_localization'

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
    maintainer='sharathnpayyadi',
    maintainer_email='sharathnp1998@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "scan_map_localizer = diff_drive_localization.scan_map_localizer:main",
            "trajectory_recorder =  diff_drive_localization.trajectory_recorder:main",
            "ekf_fusion =  diff_drive_localization.ekf_fusion:main"
        ],
    },
)
