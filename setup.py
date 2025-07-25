from setuptools import setup

package_name = 'havoc_ptz'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', [package_name + '/resource/' + package_name]),
        ('share/' + package_name + '/launch', [package_name + '/launch/ptz_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='PTZ control node with TF and prediction',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ptz_node = havoc_ptz.ptz_node:main',
        ],
    },
)
