from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'gbt_gripper_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含所有 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 包含 urdf 目录及其所有子目录下的所有文件
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        # # 包含 meshes 目录下的所有文件
        (os.path.join('share', package_name, 'meshes/visual'), glob('meshes/visual/*.*')),
        (os.path.join('share', package_name, 'meshes/collision'), glob('meshes/collision/*.*')),
        
        # # 包含 rviz 配置目录下的所有文件
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gbt',
    maintainer_email='gbt@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
