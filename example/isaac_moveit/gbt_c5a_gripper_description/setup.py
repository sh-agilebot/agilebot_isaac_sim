from setuptools import setup
import os

package_name = 'gbt_c5a_gripper_description'

# Helper function to include directory trees
def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # Standard directories to include
    standard_dirs = ['launch', 'urdf', 'meshes', 'rviz', 'config', 'scripts']
    
    for directory in standard_dirs:
        if os.path.exists(directory):
            for root, dirs, files in os.walk(directory):
                if files:
                    # Destination is share/package_name/current_relative_path
                    dest = os.path.join('share', package_name, root)
                    # Sources are the full relative paths to the files
                    sources = [os.path.join(root, f) for f in files]
                    data_files.append((dest, sources))
    
    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gbt',
    maintainer_email='gbt@todo.todo',
    description='C5A Robot Description with Robotiq 2F-140 Gripper',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
