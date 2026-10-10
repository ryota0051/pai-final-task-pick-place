from glob import glob

from setuptools import setup

package_name = 'pai_task_env'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryota0051',
    maintainer_email='35424061+ryota0051@users.noreply.github.com',
    description='CRANE+ V2 の把持・配置タスク環境',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_node = pai_task_env.task_manager_node:main',
        ],
    },
)
