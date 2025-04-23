from setuptools import find_packages, setup
from glob import glob

package_name = 'bev_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + '/config', glob('config/*.py')),
        ('share/' + '/data_process', glob('data_process/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='irtazahyder2002@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bev_server = bev_py.make_pcap_bev_video_server:main',
            'pcap_to_bev = bev_py.make_pcap_bev_video_client:main',
            'publish_bev = bev_py.rviz_with_bev:main',
            'live_bev = bev_py.live_sensor_bev:main',
        ],
    },
)
