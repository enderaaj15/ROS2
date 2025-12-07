import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'raajaction_musicplayer_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enders',
    maintainer_email='enders@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'music_server = raajaction_musicplayer_py.music_player_server:main',
            'music_client = raajaction_musicplayer_py.music_player_client:main',
        ],
    },
)
