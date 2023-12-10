from setuptools import find_packages, setup
import os, glob

package_name = 'project4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shilpa',
    maintainer_email='shilpamukhopadhyayms@gmail.com',
    description='simulator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'do_it = project4.main:main_sim',
        	'do_it_2= project4.main:main_vel',
        	'do_it_3= project4.main:main_navigate',
        	'xml_create = project4.xml_:main',
        	#'4d_sim = project4.main_4d:main_sim',
        	#'4d_vel = project4.main_4d:main_vel',
        	#'4d_nav = project4.main_4d:main_navigate',
        	'4d_sim = project4.proj_4d_backup:main_sim',
        	'4d_vel = project4.proj_4d_backup:main_vel',
        	'4d_nav = project4.proj_4d_backup:main_navigate',
        	
        ],
    },
)
