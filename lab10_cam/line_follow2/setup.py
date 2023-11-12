from setuptools import find_packages, setup

package_name = 'line_follow2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yaxjacob',
    maintainer_email='yaxjacob@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = line_follow2.pure_pursuit:main',
            'detect = line_follow2.detect:main',
            'ground_spot = line_follow2.ground_spot:main',
            'pid_run = line_follow2.pid_run:main'
        ],
    },
)
