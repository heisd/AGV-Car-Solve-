import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_web'


def recursive_data_files(install_base, src_dir):
    """Install every file under src_dir, preserving its sub-tree, into install_base/src_dir."""
    entries = []
    for root, _, files in os.walk(src_dir):
        if not files:
            continue
        install_dir = os.path.join(install_base, root)
        entries.append((install_dir, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ] + recursive_data_files(os.path.join('share', package_name), 'web'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li',
    maintainer_email='jvf36276@gmail.com',
    description='Web operator panel for the AMR fleet dispatch center (rosbridge + roslibjs).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
