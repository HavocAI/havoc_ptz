from setuptools import find_packages, setup

package_name = 'havoc_ptz'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_dir={'.': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'onvif-zeep', 'lxml', 'pygame', 'numpy', 'zeep'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='PTZ control node with TF and prediction',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ptz_node = havoc_ptz.ptz_node:main',
        ],
    },
)
