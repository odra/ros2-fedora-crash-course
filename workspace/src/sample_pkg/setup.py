from setuptools import find_packages, setup

package_name = 'sample_pkg'

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
    maintainer='lrossetti',
    maintainer_email='me@lrossetti.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node=sample_pkg.cli.simple_node:main',
            'simple_pub=sample_pkg.cli.simple_pub:main',
            'simple_sub=sample_pkg.cli.simple_sub:main',
            'sum_srv=sample_pkg.cli.sum_srv:main',
            'sum_cli=sample_pkg.cli.sum_cli:main',
            'hw_state_pub=sample_pkg.cli.hw_state_pub:main',
        ],
    },
)
