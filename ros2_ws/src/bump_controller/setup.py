from setuptools import find_packages, setup

import warnings
'''
from setuptools.commands.easy_install import EasyInstallDeprecationWarning
warnings.filterwarnings("ignore", category=EasyInstallDeprecationWarning)
'''

package_name = 'bump_controller'

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
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bump_node = bump_controller.bump_node:main'
        ],
    },
)
