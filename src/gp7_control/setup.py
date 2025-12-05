# from setuptools import find_packages, setup

# package_name = 'gp7_control'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='pankaja',
#     maintainer_email='pankajabalasooriya566@gmail.com',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )

from setuptools import setup

package_name = 'gp7_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='pankaja',
    maintainer_email='pankajabalasooriya566@gmail.com',
    description='Kinematics and control for GP7 pick-and-place assignment',
    license='BSB',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'pick_and_place = gp7_control.pick_and_place_node:main',
        'test_movement = gp7_control.test_movement_node:main',
        'simple_joint_move = gp7_control.simple_joint_move_node:main',
    ],
},
)
