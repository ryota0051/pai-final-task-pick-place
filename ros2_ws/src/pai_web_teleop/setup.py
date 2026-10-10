from glob import glob

from setuptools import setup

package_name = 'pai_web_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryota0051',
    maintainer_email='35424061+ryota0051@users.noreply.github.com',
    description='CRANE+ V2 のブラウザ手先テレオペと録画ノード',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ee_teleop_node = pai_web_teleop.ee_teleop_node:main',
            'recorder_node = pai_web_teleop.recorder_node:main',
        ],
    },
)
