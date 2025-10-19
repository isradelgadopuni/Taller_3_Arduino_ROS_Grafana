from setuptools import setup

package_name = 'sensor_program'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Israel Delgado',
    maintainer_email='isra-5-delgado@outlook.com',
    description='Nodo sensor + lector + plotter (reto).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = sensor_program.sensor_node:main',
            'reader_node = sensor_program.reader_node:main',
            'plotter_node = sensor_program.plotter_node:main',
        ],
    },
)
