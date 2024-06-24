from setuptools import setup

package_name = 'chase_object_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tofunmi Sodimu',
    maintainer_email='tofsodimu@gmail.com',
    description='chase an object',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = chase_object_python.detect_object:main',
            'listener_1 = chase_object_python.get_object_range:main',
            'listener_2 = chase_object_python.chase_object:main',
            'listener_3 = chase_object_python.get_range:main',
        ],
    },
)
