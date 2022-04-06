from setuptools import setup
import os
package_name = 'ipd_calib'
from glob import glob

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben',
    maintainer_email='ben.e.rautio@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ipd_cal_checker = ipd_calib.ipd_cal_checker_node:main',
            'ipd_ext_checker = ipd_calib.ipd_ext_checker_node:main'
        ],
    },
)
