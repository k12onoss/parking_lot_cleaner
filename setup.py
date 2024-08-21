import os

from glob import glob, iglob
from setuptools import find_packages, setup

package_name = 'parking_lot_cleaner'

def models_dir(directory) -> list[tuple[str, list[str]]]:
    pairs: list[tuple[str, list[str]]] = []
    helper(directory, pairs)
    return pairs

def helper(directory: str, pairs: list[tuple[str, list[str]]]) -> None:
    pairs.append((os.path.join('share', package_name, directory), glob(os.path.join(directory, '*'))))

    for pair in pairs:
        share_path, paths = pair
        for path in paths:
            if os.path.isdir(path):
                paths.remove(path)
                helper(path, pairs)

models = models_dir('models')

data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
        (os.path.join('share', package_name, 'trained_model'), glob(os.path.join('trained_model', '*'))),
]
data_files.extend(models)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ujjwal',
    maintainer_email='ujjwal2508@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hover_algorithm.py = parking_lot_cleaner.hover_algorithm:main',
            'garbage_spawner.py = parking_lot_cleaner.garbage_spawner:main',
            'garbage_deleter.py = parking_lot_cleaner.garbage_deleter:main',
            'image_saver.py = parking_lot_cleaner.image_saver:main',
            'garbage_detector.py = parking_lot_cleaner.garbage_detector:main',
            'job_allocator.py = parking_lot_cleaner.job_allocator:main',
            'initial_pose_publisher.py = parking_lot_cleaner.initial_pose_publisher:main',
            'nav2_goal_sender.py = parking_lot_cleaner.nav2_goal_sender:main'
        ],
    },
)
