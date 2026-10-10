from glob import glob

from setuptools import setup

package_name = 'crane_plus_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='ryota0051',
    maintainer_email='35424061+ryota0051@users.noreply.github.com',
    description='CRANE+ V2 の順運動学・解析的逆運動学ライブラリ',
    license='Apache-2.0',
    tests_require=['pytest'],
)
