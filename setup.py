from setuptools import find_packages, setup

package_name = 'turtle_scanner_Emmanuella'

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
    maintainer='user',
    maintainer_email='emmanuellabangali6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
   'spawn_target = turtle_scanner_Emmanuella.spawn_target:main',
   'turtle_scanner_node = turtle_scanner_Emmanuella.turtle_scanner_node:main',
],
    },
)
