from setuptools import find_packages, setup

package_name = 'Rob2002_development'

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
    maintainer='ros',
    maintainer_email='matthewandrews31@yahoo.co.uk',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mover = Rob2002_development.mover_basic:main',
            'colour = Rob2002_development.detector_basic:main',
            'spin = Rob2002_development.spinner:main',
        ],
    },
)
