from setuptools import find_packages, setup

package_name = 'wili_io'

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
    maintainer='Shinagawa Kazemaru',
    maintainer_email='marukazemaru0@gmail.com',
    description='input and output for WiLI(Where is a Lost Item)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'db_proxy=wili_io.db_proxy:main'
        ],
    },
)
