from setuptools import find_packages, setup

package_name = 'py_armnode'

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
    maintainer='oretmj',
    maintainer_email='oreste.mosciatti@uc.cl',
    description='Python Node for Ned2 arm control via pyniryo library',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = py_armnode.armnode:main',
        ],
    },
)
