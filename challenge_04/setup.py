import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'challenge_04'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),
	    glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puzzlebot',
    maintainer_email='puzzlebot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller10= challenge_04.controller10:main',
            'odometry = challenge_04.odometry:main',
            'path3 = challenge_04.path3:main',
            'cameraPub = challenge_04.cameraPub:main',
            'susImage4 = challenge_04.susImage4:main',
            'pathCustom = challenge_04.pathCustom:main',
            'camPath4 = challenge_04.camPath4:main'
        ],
    },
)
