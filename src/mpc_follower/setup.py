from setuptools import find_packages, setup

package_name = 'mpc_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_all.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hafsa',
    maintainer_email='hafsa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_follower_node = mpc_follower.mpc_controller_node:main'
        ],
    },
)
