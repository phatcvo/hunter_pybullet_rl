from setuptools import find_packages, setup

package_name = 'pybullet_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', [
            'urdf/hunter_model.urdf'
        ]),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phat',
    maintainer_email='vophat0607@gmail.com',
    description='PyBullet simulator for Hunter robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_node = pybullet_sim.sim_node:main',
        ],
    },
)
