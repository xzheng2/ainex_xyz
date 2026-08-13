from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'xyz_perception',
        'xyz_perception.apriltag',
        'xyz_perception.color',
        'xyz_perception.depth_nav',
        'xyz_perception.yolo',
    ],
    package_dir={'': 'src'},
)
setup(**d)
