import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'lsm_fms_web'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*.*')),
        (os.path.join('share', package_name, 'web/assets'), glob('web/assets/*.*')),
        (os.path.join('share', package_name, 'web/config'), glob('web/config/*.*')),
        (os.path.join('share', package_name, 'web/css'), glob('web/css/*.*')),
        (os.path.join('share', package_name, 'web/js'), glob('web/js/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kms',
    maintainer_email='kms@todo.todo',
    description='Decoupled Web Command Center for LSM FMS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = lsm_fms_web.web_server:main',
            'route_server = lsm_fms_web.lsm_fms_route_server:main',
            'telemetry_bridge = lsm_fms_web.telemetry_bridge:main',
        ],
    },
)
