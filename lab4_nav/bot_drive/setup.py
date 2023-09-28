from setuptools import find_packages, setup

package_name = 'bot_drive'

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
            'circle_drive = bot_drive.circle_drive:main',
	    'bot_monitor = bot_drive.bot_monitor:main',
            'square_drive = bot_drive.square_drive:main'
        ],
    },
)
