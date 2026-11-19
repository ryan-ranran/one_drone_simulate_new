from setuptools import setup

package_name = 'apriltag_align'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='one_drone',
    maintainer_email='team@example.com',
    description='Downward AprilTag visual landing alignment',
    license='MIT',
    entry_points={'console_scripts': ['apriltag_align = apriltag_align.detector_node:main']},
)
