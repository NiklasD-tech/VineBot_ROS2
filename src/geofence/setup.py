from setuptools import setup

package_name = 'geofence'

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
    maintainer='irobot',
    maintainer_email='niklas.dettelbacher@study.thws.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointInPolygon = geofence.pointInPolygon:main',
            'geofence_node = geofence.geofence_node:main'  # Neuer Eintrag für den GPS-Subscriber
        ],
    },
)
