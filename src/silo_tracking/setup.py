from setuptools import setup

package_name = 'silo_tracking'

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
    maintainer='robocon',
    maintainer_email='eangsokheang23@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "silo_red = silo_tracking.silo_red:main",
            "silo_blue = silo_tracking.silo_blue:main"
        ],
    },
)
