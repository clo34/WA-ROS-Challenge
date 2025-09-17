from setuptools import setup

package_name = 'merge_arrays'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]) if False else (),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Merges two sorted Int32MultiArray inputs into a sorted output.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'merge_arrays_node = merge_arrays.merge_arrays_node:main',
        ],
    },
)
