from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'group2_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Install maps
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        # Install RViz config
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maxykumd , namfacchetti',
    maintainer_email='maxyk@umd.edu, gfacchet@terpmail.umd.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # BT entry point
            'search_and_rescue_exe = group2_final.scripts.main_search_and_rescue:main',
            # Service servers 
            'detect_survivor_server_exe = group2_final.scripts.main_detect_survivor_server:main',
            'report_survivor_server_exe = group2_final.scripts.main_report_survivor_server:main',
        ],
    },
)
